#include <NIDAQmx.h>
#include <tinyxml2.h>

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <dirent.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

using Matrix6x6 = std::array<std::array<double, 6>, 6>;
using Vector6 = std::array<double, 6>;

constexpr int NUM_CHANNELS = 6;
constexpr double SAMPLING_FREQUENCY = 500.0;
constexpr int SAMPLES_PER_READ = 1;
constexpr int DAQ_BUFFER_SIZE = 5000;

const std::string ROOT_DIR = "/home/hela/Desktop/ft_libbarrett";
const std::string DATA_DIR = ROOT_DIR + "/data";
const std::string CAL_PATH = ROOT_DIR + "/cal/FT9235/FT9235.cal";

std::atomic<bool> running{true};
std::atomic<bool> recording{false};

struct FTLogRow {
    uint64_t sampleIndex;

    double hardwareTime;
    double dtHw;

    double readTime;
    double dtRead;

    double batchTime;
    double dtBatch;

    double fx, fy, fz;
    double tx, ty, tz;
};

std::queue<FTLogRow> logQueue;
std::mutex logMutex;
std::condition_variable logCv;

std::ofstream csvFile;
std::mutex fileMutex;

void signalHandler(int)
{
    running = false;
    recording = false;
    logCv.notify_all();
}

void enableRealtimeForCurrentThread(int priority)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "Warning: mlockall failed. Try running with sudo.\n";
    }

    sched_param param{};
    param.sched_priority = priority;

    if (sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
        std::cerr << "Warning: could not set SCHED_FIFO. Try running with sudo.\n";
    }
}

class TerminalRawMode {
public:
    TerminalRawMode()
    {
        tcgetattr(STDIN_FILENO, &oldSettings_);

        termios newSettings = oldSettings_;
        newSettings.c_lflag &= ~(ICANON | ECHO);

        tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);
    }

    ~TerminalRawMode()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings_);
    }

private:
    termios oldSettings_{};
};

bool keyboardHit()
{
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    return select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0;
}

char getKey()
{
    if (keyboardHit()) {
        char c;
        if (read(STDIN_FILENO, &c, 1) > 0) {
            return c;
        }
    }

    return '\0';
}

void checkDaqError(int32 error, TaskHandle taskHandle)
{
    if (error < 0) {
        char errBuff[2048] = {'\0'};
        DAQmxGetExtendedErrorInfo(errBuff, sizeof(errBuff));

        if (taskHandle != nullptr) {
            DAQmxStopTask(taskHandle);
            DAQmxClearTask(taskHandle);
        }

        throw std::runtime_error(errBuff);
    }
}

Matrix6x6 loadAtiCalibration(const std::string& filepath)
{
    tinyxml2::XMLDocument doc;

    if (doc.LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Could not load calibration file: " + filepath);
    }

    auto* root = doc.FirstChildElement("FTSensor");
    if (!root) {
        throw std::runtime_error("Could not find <FTSensor> tag.");
    }

    auto* calibration = root->FirstChildElement("Calibration");
    if (!calibration) {
        throw std::runtime_error("Could not find <Calibration> tag.");
    }

    Matrix6x6 matrix{};
    int row = 0;

    for (auto* axis = calibration->FirstChildElement("Axis");
         axis != nullptr;
         axis = axis->NextSiblingElement("Axis")) {

        if (row >= 6) {
            break;
        }

        const char* valuesText = axis->Attribute("values");
        if (!valuesText) {
            throw std::runtime_error("Axis has no values attribute.");
        }

        double scale = 1.0;
        axis->QueryDoubleAttribute("scale", &scale);

        std::stringstream ss(valuesText);

        for (int col = 0; col < 6; ++col) {
            double value = 0.0;
            ss >> value;
            matrix[row][col] = value / scale;
        }

        row++;
    }

    if (row != 6) {
        throw std::runtime_error("Calibration matrix does not have 6 rows.");
    }

    return matrix;
}

Vector6 multiplyMatrixVector(const Matrix6x6& M, const Vector6& v)
{
    Vector6 result{};

    for (int i = 0; i < 6; ++i) {
        result[i] = 0.0;

        for (int j = 0; j < 6; ++j) {
            result[i] += M[i][j] * v[j];
        }
    }

    return result;
}

std::string makeRecordingFilename()
{
    mkdir(DATA_DIR.c_str(), 0777);

    int fileCount = 0;

    DIR* dir = opendir(DATA_DIR.c_str());

    if (dir != nullptr) {
        struct dirent* entry;

        while ((entry = readdir(dir)) != nullptr) {
            std::string name = entry->d_name;

            if (name.size() > 4 && name.substr(name.size() - 4) == ".csv") {
                fileCount++;
            }
        }

        closedir(dir);
    }

    std::ostringstream filename;
    filename << DATA_DIR << "/ft_data_" << fileCount << ".csv";

    return filename.str();
}

void pushLogRow(const FTLogRow& row)
{
    {
        std::lock_guard<std::mutex> lock(logMutex);
        logQueue.push(row);
    }

    logCv.notify_one();
}

void loggerThreadFunction()
{
    std::vector<FTLogRow> localRows;
    localRows.reserve(1000);

    while (running || !logQueue.empty()) {
        {
            std::unique_lock<std::mutex> lock(logMutex);

            logCv.wait(lock, [] {
                return !logQueue.empty() || !running;
            });

            while (!logQueue.empty()) {
                localRows.push_back(logQueue.front());
                logQueue.pop();
            }
        }

        {
            std::lock_guard<std::mutex> fileLock(fileMutex);

            if (csvFile.is_open()) {
                for (const auto& row : localRows) {
                    csvFile << std::fixed << std::setprecision(9)
                            << row.sampleIndex << ","
                            << row.hardwareTime << ","
                            << row.dtHw << ","
                            << row.readTime << ","
                            << row.dtRead << ","
                            << row.batchTime << ","
                            << row.dtBatch << ","
                            << row.fx << ","
                            << row.fy << ","
                            << row.fz << ","
                            << row.tx << ","
                            << row.ty << ","
                            << row.tz << "\n";
                }
            }
        }

        localRows.clear();
    }
}

void daqThreadFunction(
    const Matrix6x6& calibrationMatrix,
    const Vector6& bias,
    TaskHandle taskHandle
)
{
    enableRealtimeForCurrentThread(80);

    std::vector<double> rawData(NUM_CHANNELS * SAMPLES_PER_READ);
    int32 samplesRead = 0;

    auto recordingStartTime = std::chrono::steady_clock::now();

    uint64_t sampleIndex = 0;

    double previousReadTime = 0.0;
    double previousBatchTime = 0.0;

    while (running) {
        int32 error = DAQmxReadAnalogF64(
            taskHandle,
            SAMPLES_PER_READ,
            1.0,
            DAQmx_Val_GroupByChannel,
            rawData.data(),
            static_cast<uInt32>(rawData.size()),
            &samplesRead,
            nullptr
        );

        checkDaqError(error, taskHandle);

        auto batchNow = std::chrono::steady_clock::now();

        if (!recording) {
            recordingStartTime = batchNow;
            sampleIndex = 0;
            previousReadTime = 0.0;
            previousBatchTime = 0.0;
            continue;
        }

        double batchTime = std::chrono::duration<double>(
            batchNow - recordingStartTime
        ).count();

        double dtBatch = 0.0;
        if (sampleIndex > 0) {
            dtBatch = batchTime - previousBatchTime;
        }
        previousBatchTime = batchTime;

        for (int s = 0; s < samplesRead; ++s) {
            Vector6 netVoltage{};

            for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
                netVoltage[ch] = rawData[ch * samplesRead + s] - bias[ch];
            }

            Vector6 ft = multiplyMatrixVector(calibrationMatrix, netVoltage);

            auto readNow = std::chrono::steady_clock::now();

            double readTime = std::chrono::duration<double>(
                readNow - recordingStartTime
            ).count();

            double hardwareTime =
                static_cast<double>(sampleIndex) / SAMPLING_FREQUENCY;

            double dtHw = 1.0 / SAMPLING_FREQUENCY;

            double dtRead = 0.0;
            if (sampleIndex > 0) {
                dtRead = readTime - previousReadTime;
            }
            previousReadTime = readTime;

            FTLogRow row{
                sampleIndex,
                hardwareTime,
                dtHw,
                readTime,
                dtRead,
                batchTime,
                dtBatch,
                ft[0], ft[1], ft[2],
                ft[3], ft[4], ft[5]
            };

            pushLogRow(row);

            sampleIndex++;
        }
    }
}

int main()
{
    std::signal(SIGINT, signalHandler);

    const std::string channelString = "Dev1/ai0:5";

    TaskHandle taskHandle = nullptr;

    try {
        TerminalRawMode terminalMode;

        Matrix6x6 calibrationMatrix = loadAtiCalibration(CAL_PATH);

        int32 error = DAQmxCreateTask("", &taskHandle);
        checkDaqError(error, taskHandle);

        error = DAQmxCreateAIVoltageChan(
            taskHandle,
            channelString.c_str(),
            "",
            DAQmx_Val_Diff,
            -10.0,
            10.0,
            DAQmx_Val_Volts,
            nullptr
        );
        checkDaqError(error, taskHandle);

        error = DAQmxCfgSampClkTiming(
            taskHandle,
            "",
            SAMPLING_FREQUENCY,
            DAQmx_Val_Rising,
            DAQmx_Val_ContSamps,
            DAQ_BUFFER_SIZE
        );
        checkDaqError(error, taskHandle);

        error = DAQmxStartTask(taskHandle);
        checkDaqError(error, taskHandle);

        constexpr int numTareSamples = 500;

        std::vector<double> tareData(NUM_CHANNELS * numTareSamples);
        int32 tareSamplesRead = 0;

        error = DAQmxReadAnalogF64(
            taskHandle,
            numTareSamples,
            5.0,
            DAQmx_Val_GroupByChannel,
            tareData.data(),
            static_cast<uInt32>(tareData.size()),
            &tareSamplesRead,
            nullptr
        );
        checkDaqError(error, taskHandle);

        Vector6 bias{};

        for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
            double sum = 0.0;

            for (int s = 0; s < tareSamplesRead; ++s) {
                sum += tareData[ch * tareSamplesRead + s];
            }

            bias[ch] = sum / tareSamplesRead;
        }

        std::cout << "Ready.\n";
        std::cout << "r: start recording\n";
        std::cout << "x: stop recording\n";
        std::cout << "q: quit\n";

        std::thread loggerThread(loggerThreadFunction);

        std::thread daqThread(
            daqThreadFunction,
            std::cref(calibrationMatrix),
            std::cref(bias),
            taskHandle
        );

        std::string recordingFilename;

        while (running) {
            char key = getKey();

            if (key == 'r' && !recording) {
                recordingFilename = makeRecordingFilename();

                {
                    std::lock_guard<std::mutex> fileLock(fileMutex);

                    csvFile.open(recordingFilename.c_str());

                    if (!csvFile.is_open()) {
                        throw std::runtime_error("Could not open CSV file.");
                    }

                    csvFile << "sample_index,"
                            << "hardware_time,dt_hw,"
                            << "read_time,dt_read,"
                            << "batch_time,dt_batch,"
                            << "Fx,Fy,Fz,Tx,Ty,Tz\n";
                }

                recording = true;
                std::cout << "Recording started.\n";
            }

            else if (key == 'x' && recording) {
                recording = false;

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                {
                    std::lock_guard<std::mutex> fileLock(fileMutex);

                    if (csvFile.is_open()) {
                        csvFile.flush();
                        csvFile.close();
                    }
                }

                std::cout << "Recording stopped.\n";
                std::cout << "Saved to: " << recordingFilename << "\n";
            }

            else if (key == 'q') {
                running = false;
                recording = false;
                logCv.notify_all();
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (daqThread.joinable()) {
            daqThread.join();
        }

        logCv.notify_all();

        if (loggerThread.joinable()) {
            loggerThread.join();
        }

        {
            std::lock_guard<std::mutex> fileLock(fileMutex);

            if (csvFile.is_open()) {
                csvFile.flush();
                csvFile.close();
            }
        }

        DAQmxStopTask(taskHandle);
        DAQmxClearTask(taskHandle);

    } catch (const std::exception& e) {
        running = false;
        recording = false;
        logCv.notify_all();

        if (csvFile.is_open()) {
            csvFile.flush();
            csvFile.close();
        }

        if (taskHandle != nullptr) {
            DAQmxStopTask(taskHandle);
            DAQmxClearTask(taskHandle);
        }

        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}