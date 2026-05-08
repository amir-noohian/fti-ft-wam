#include <NIDAQmx.h>
#include <tinyxml2.h>

#include <iostream>
#include <vector>
#include <array>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>
#include <csignal>

static bool running = true;

void signalHandler(int)
{
    running = false;
}

using Matrix6x6 = std::array<std::array<double, 6>, 6>;
using Vector6 = std::array<double, 6>;

Matrix6x6 loadAtiCalibration(const std::string& filepath)
{
    tinyxml2::XMLDocument doc;

    if (doc.LoadFile(filepath.c_str()) != tinyxml2::XML_SUCCESS) {
        throw std::runtime_error("Could not load calibration file: " + filepath);
    }

    tinyxml2::XMLElement* root = doc.FirstChildElement("FTSensor");
    if (!root) {
        throw std::runtime_error("Could not find <FTSensor> tag.");
    }

    const char* serial = root->Attribute("Serial");
    std::cout << "Loaded calibration for sensor: "
              << (serial ? serial : "Unknown") << std::endl;

    tinyxml2::XMLElement* calibration = root->FirstChildElement("Calibration");
    if (!calibration) {
        throw std::runtime_error("Could not find <Calibration> tag.");
    }

    Matrix6x6 matrix{};
    int row = 0;

    for (tinyxml2::XMLElement* axis = calibration->FirstChildElement("Axis");
         axis != nullptr;
         axis = axis->NextSiblingElement("Axis")) {

        if (row >= 6) {
            break;
        }

        const char* valuesText = axis->Attribute("values");
        double scale = 1.0;
        axis->QueryDoubleAttribute("scale", &scale);

        if (!valuesText) {
            throw std::runtime_error("Axis does not contain values.");
        }

        std::stringstream ss(valuesText);

        for (int col = 0; col < 6; ++col) {
            double value;
            ss >> value;

            // Same as Python: scaled_row = value / scale
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

void checkDaqError(int32 error, TaskHandle taskHandle)
{
    if (error < 0) {
        char errBuff[2048] = {'\0'};
        DAQmxGetExtendedErrorInfo(errBuff, 2048);

        if (taskHandle != nullptr) {
            DAQmxStopTask(taskHandle);
            DAQmxClearTask(taskHandle);
        }

        throw std::runtime_error(errBuff);
    }
}

int main()
{
    std::signal(SIGINT, signalHandler);

    const std::string calPath =
        "/home/hela/Desktop/ATIFT/cal/FT9235/FT9235.cal";

    const std::string channelString = "Dev2/ai0:5";

    try {
        Matrix6x6 calibrationMatrix = loadAtiCalibration(calPath);

        TaskHandle taskHandle = nullptr;
        int32 error = 0;

        std::cout << "Connecting to NI-DAQmx..." << std::endl;

        error = DAQmxCreateTask("", &taskHandle);
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

        error = DAQmxStartTask(taskHandle);
        checkDaqError(error, taskHandle);

        std::cout << "\nZeroing sensor. Make sure nothing is touching it..."
                  << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(2));

        constexpr int numChannels = 6;
        constexpr int numSamples = 100;

        std::vector<double> tareData(numChannels * numSamples);
        int32 samplesRead = 0;

        error = DAQmxReadAnalogF64(
            taskHandle,
            numSamples,
            10.0,
            DAQmx_Val_GroupByChannel,
            tareData.data(),
            tareData.size(),
            &samplesRead,
            nullptr
        );
        checkDaqError(error, taskHandle);

        Vector6 bias{};

        for (int ch = 0; ch < numChannels; ++ch) {
            double sum = 0.0;

            for (int s = 0; s < samplesRead; ++s) {
                sum += tareData[ch * samplesRead + s];
            }

            bias[ch] = sum / samplesRead;
        }

        std::cout << "Bias recorded. Starting live stream. Press Ctrl+C to stop.\n"
                  << std::endl;

        while (running) {
            double rawData[numChannels];
            int32 read = 0;

            error = DAQmxReadAnalogF64(
                taskHandle,
                1,
                10.0,
                DAQmx_Val_GroupByChannel,
                rawData,
                numChannels,
                &read,
                nullptr
            );
            checkDaqError(error, taskHandle);

            Vector6 netVoltage{};

            for (int i = 0; i < numChannels; ++i) {
                netVoltage[i] = rawData[i] - bias[i];
            }

            Vector6 ft = multiplyMatrixVector(calibrationMatrix, netVoltage);

            std::cout << "\r\033[K"
                      << "FT9235 | "
                      << "Fx: " << ft[0] << " N | "
                      << "Fy: " << ft[1] << " N | "
                      << "Fz: " << ft[2] << " N || "
                      << "Tx: " << ft[3] << " Nm | "
                      << "Ty: " << ft[4] << " Nm | "
                      << "Tz: " << ft[5] << " Nm"
                      << std::flush;

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::cout << "\nStopping data acquisition safely..." << std::endl;

        DAQmxStopTask(taskHandle);
        DAQmxClearTask(taskHandle);

    } catch (const std::exception& e) {
        std::cerr << "\nError: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}