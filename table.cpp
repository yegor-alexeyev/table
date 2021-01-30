#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <utility>
#include <iomanip>
#include <cstdlib>
#include <unordered_map>
#include <unordered_set>

enum class WorkMode {
    drive,
    workdrive,
    result,
    workdrivesymc,
    resultsymc
};

WorkMode parse_work_mode(const std::string& mode) {
    if (mode == "drive") {
        return WorkMode::drive;
    }
    if (mode == "workdrive") {
        return WorkMode::workdrive;
    }
    if (mode == "result") {
        return WorkMode::result;
    }
    if (mode == "workdrivesymc") {
        return WorkMode::workdrivesymc;
    }
    if (mode == "resultsymc") {
        return WorkMode::resultsymc;
    }
    throw std::runtime_error("Unknown mode: " + mode + ". Supported modes: drive, workdrive, result, workdrivesymc, resultsymc.");
}

int main(int argc, const char* argv[])
{
    try {

        if (argc < 3)
        {
            std::cerr << "Usage(order of arguments is important): " << argv[0] << " " << "INPUT OUTPUT [drive | workdrive] [dampening-factor=1.0] [path-to-osrm-file=map_data\\germany-latest.osrm] " << "\n";
            std::cerr << "Usage(order of arguments is important): " << argv[0] << " " << "INPUT OUTPUT result INPUT-RESULT-FILE [dampening-factor=1.0] [path-to-osrm-file=map_data\\germany-latest.osrm] " << "\n";
            std::cerr << "Usage(order of arguments is important): " << argv[0] << " " << "INPUT OUTPUT workdrivesymc [worktime-limit-in-minutes=2400] [dampening-factor=1.0] [path-to-osrm-file=map_data\\germany-latest.osrm] " << "\n";
            std::cerr << "Usage(order of arguments is important): " << argv[0] << " " << "INPUT OUTPUT resultsymc OP-SOLVER-SOLUTION-FILE DISTANCE-MATRIX-INPUT-FILE [OUTPUT-JS-DEFINITIONS-FILENAME]" << "\n";
            std::cerr << "Example: " << argv[0] << " " << "input.txt output.result.txt result input.result.txt 1.0 map_data\\germany-latest.osrm " << "\n";
            std::cerr << "Example: " << argv[0] << " " << "input.txt output.txt resultsymc solver.o-148535.sol workdrivesymc.out.txt ..\\custom-markers-reduced\\features.js " << "\n";
            return EXIT_FAILURE;
        }



        const std::string inputFilename(argv[1]);
        const std::string outputFilename(argv[2]);

        WorkMode work_mode = argc < 4 ? WorkMode::drive : parse_work_mode(argv[3]);

        int arg_offset = work_mode == WorkMode::result || work_mode == WorkMode::resultsymc || work_mode == WorkMode::workdrivesymc ? 1 : 0;

        std::string worktime_limit_in_minutes = "2400";
        if (work_mode == WorkMode::workdrivesymc && argc >= 5 ) {
            worktime_limit_in_minutes = argv[4];
        }

        std::ifstream inputFile(inputFilename);
        if (!inputFile.is_open()) {
            throw std::runtime_error("error opening input file " + inputFilename);
        }

        using namespace osrm;
        TableParameters params;
        //std::vector<double> workDuration;
        std::vector<int> jobRowToId;
        std::unordered_map<int, double> workDurationMap;
        std::unordered_map<int, double> score1Map;



        std::ofstream outputFile(outputFilename);
        if (!outputFile.is_open()) {
            throw std::runtime_error("error writing output file " + outputFilename);
        }

        std::string line;
        while (std::getline(inputFile, line)) {
            {
                if (work_mode != WorkMode::result && work_mode != WorkMode::workdrivesymc && work_mode != WorkMode::resultsymc) {
                    outputFile << line << "\n";
                }

                std::istringstream lineCheckStream(line);
                char firstSymbol;
                lineCheckStream >> firstSymbol;
                if (!std::isdigit(firstSymbol)) {
                    std::cerr << "skipping line: " << line << "\n";
                    continue;
                }
            }

            std::istringstream lineStream(line);
            char separator1;
            char separator2;
            char separator3;
            char separator4;
            int uniqueJobID;
            double latitude;
            double longitude;
            double score1;
            double score2;

            lineStream >> uniqueJobID >> separator1 >> latitude >> separator2 >> longitude;
            lineStream >> separator3 >> score1 >> separator4 >> score2;

            if (separator1 != ',' || separator2 != ',' || separator3 != ',' || separator4 != ',') {
                throw std::runtime_error("invalid input line: " + line);
            }

            params.coordinates.push_back({ util::FloatLongitude{longitude}, util::FloatLatitude{latitude} });
            //workDuration.push_back(score2);

            jobRowToId.push_back(uniqueJobID);
            workDurationMap[uniqueJobID] = score2;
            score1Map[uniqueJobID] = score1;
        }

        if (work_mode == WorkMode::resultsymc) {
            if (argc < 6) {
                std::cerr << "Error, an nsufficient number of arguments were supplied for the resultsymc mode" << '\n';
                std::cerr << "Usage(order of arguments is important): " << argv[0] << " " << "INPUT OUTPUT resultsymc OP-SOLVER-SOLUTION-FILE DISTANCE-MATRIX-INPUT-FILE [OUTPUT-JS-DEFINITIONS-FILENAME]" << "\n";
                std::cerr << "Example: " << argv[0] << " " << "input.txt output.txt resultsymc solver.o-148535.sol workdrivesymc.out.txt ..\\custom-markers-reduced\\features.js " << "\n";
                return EXIT_FAILURE;
            }

            const std::string matrixInputFilename = argv[5];
            std::ifstream matrixInputFile(matrixInputFilename);
            if (!matrixInputFile.is_open()) {
                throw std::runtime_error("error opening distance matrix input file " + matrixInputFilename);
            }
            std::vector< std::vector<int> > dampedDrivingTimeFromToMinutes;
            do {
                std::getline(matrixInputFile, line);
                std::cout << "SKIPPED distance matrix input line: " << line << std::endl;
            } while (line != "EDGE_WEIGHT_SECTION");


            for (size_t row = 0; row < jobRowToId.size(); row++) {
                std::string matrixRowLine;
                std::getline(matrixInputFile, matrixRowLine);

                std::istringstream lineStream(matrixRowLine);
                std::vector<int> matrixRow;
                std::copy(std::istream_iterator<int>(lineStream), std::istream_iterator<int>(), std::back_inserter(matrixRow));

                for (size_t column = 0; column < matrixRow.size(); column++) {
                    const double workFromDuration = workDurationMap.at(jobRowToId.at(row));
                    const double workToDuration = workDurationMap.at(jobRowToId.at(column));
                    matrixRow[column] -= workFromDuration/2.0;
                    matrixRow[column] -= workToDuration/2.0;
                }

                dampedDrivingTimeFromToMinutes.push_back(matrixRow);
            }

            const std::string resultInputFilename = argv[4];
            std::ifstream resultInputFile(resultInputFilename);
            if (!resultInputFile.is_open()) {
                throw std::runtime_error("error opening result input file " + resultInputFilename);
            }

            std::string line;
            do {
                std::getline(resultInputFile, line);
                std::cout << "SKIPPED result input line: " << line << std::endl;
            } while (line != "NODE_SEQUENCE_SECTION");
            //output results file


            std::vector<int> selectedJobsRowIndex; 

            std::string stopListLine;
            int i = 0;
            double totalScore1 = 0;
            double totalWorkDuration = 0;
            while (true) {
                std::getline(resultInputFile, line);
                if (line == "-1") {
                    break;
                }
                std::istringstream lineStream(line);
                int row_index_one_based;
                lineStream >> row_index_one_based;
                int row_index = row_index_one_based - 1;
                int station_id = jobRowToId.at(row_index);
                selectedJobsRowIndex.push_back(row_index);
                totalScore1 += score1Map.at(station_id);
                totalWorkDuration += workDurationMap.at(station_id);
                outputFile << station_id;
                outputFile << " ";
                i++;
            }
            outputFile << "1 "; //TODO CHECK: final_station_id=1  is consistent with java solver output, but really there should be 0
            outputFile << '\n';
            outputFile << totalScore1 << '\n';
            outputFile << totalWorkDuration << '\n';

            double dampedDrivingTimeSumMinutes = 0;
            for (size_t i = 0; i < selectedJobsRowIndex.size(); i++) {
                int from_station_row_index = selectedJobsRowIndex[i];
                int to_station_row_index = i == selectedJobsRowIndex.size() - 1 ? selectedJobsRowIndex.front() : selectedJobsRowIndex[i+1];

                if (from_station_row_index < to_station_row_index) { // hack because dampedDrivingTimeFromToMinutes contains triangular matrix form
                    std::swap(from_station_row_index, to_station_row_index);
                }

                dampedDrivingTimeSumMinutes += dampedDrivingTimeFromToMinutes.at(from_station_row_index).at(to_station_row_index);
            }

            outputFile << std::lround(dampedDrivingTimeSumMinutes) << '\n';


            if (argc >= 7) {
                const std::string js_definitions_filename = argv[6];

                std::ofstream js_file(js_definitions_filename);
                if (!js_file.is_open()) {
                    throw std::runtime_error("error writing is definitions file " + js_definitions_filename);
                }

                js_file << std::fixed << std::setprecision(7);

                js_file << "const features = [" << std::endl;
                double min_size = 15.0;
                double max_size = 16.0;


                for (size_t i = 0; i < jobRowToId.size(); i++) {
                    const int job_id = jobRowToId.at(i);
                    if ( job_id == 0 || job_id == 1) {
                        continue;
                    }

                    const double priority = score1Map.at(job_id);
                    const int duration = workDurationMap.at(job_id);
                    if (duration < 1) {
                        continue;
                    }
                    double size2_double = std::sqrt(priority/duration)*8.0;
                    if (size2_double < min_size) {
                        min_size = size2_double;
                    }
                    if (size2_double > max_size) {
                        max_size = size2_double;
                    }
                }
                std::cout << std::endl << "min " << min_size << " " << "max " << max_size << std::endl;
                for (size_t i = 0; i < jobRowToId.size(); i++) {
                    const double latitude = params.coordinates.at(i).lat.__value/1000000.0;
                    const double longitude = params.coordinates.at(i).lon.__value/1000000.0;
                    const int job_id = jobRowToId.at(i);
                    const double priority = score1Map.at(job_id);
                    const int duration = workDurationMap.at(job_id);

                    const bool is_selected = std::find(selectedJobsRowIndex.begin(), selectedJobsRowIndex.end(), i) != selectedJobsRowIndex.end();

                    std::string type =  is_selected ? "active" : "inactive";
                    int size1;
                    int size2;
                    if (job_id == 0 || job_id == 1) {
                        type = "home";
                        size1 = 20;
                        size2 = 20;
                    } else {
                        if (duration < 1) {
                            size1 = 30;
                            size2 = 30;
                        } else {
                            double size2_double = std::sqrt(priority/duration)*8.0;
                            size1 = std::lround((size2_double - min_size)*(30.0 - 10.0)/(max_size - min_size) + 10.0);
                            size2 = std::lround(size2_double);
                        }
                        if (size2 > 30) {
                            size2 = 30;
                        }
                        if (size2 < 10) {
                            size2 = 10;
                        }
                    }
                    if ( i != 0) {
                        js_file << "," << std::endl;
                    }

                    js_file << "\t{ ";
                    js_file << "latitude: "     << latitude                        << ", ";
                    js_file << "longitude: "     << longitude                        << ", ";
                    js_file << "type: \""     << type                        << "\", ";
                    js_file << "title: \""    << job_id               << "\", ";
                    js_file << "priority: " << priority << ", ";
                    js_file << "duration: " << duration << ", ";
                    js_file << "size1: " << size1 << ", ";
                    js_file << "size2: " << size2 << "";
                    js_file << " }";
                }
                js_file << std::endl << "];" << std::endl;

                js_file <<  std::endl;

                js_file << "const triangleCoords2 = [" << std::endl;
                for (size_t i = 0; i < selectedJobsRowIndex.size(); i++) {
                    const double latitude = params.coordinates.at(selectedJobsRowIndex[i]).lat.__value/1000000.0;
                    const double longitude = params.coordinates.at(selectedJobsRowIndex[i]).lon.__value/1000000.0;
                    if ( i != 0) {
                        js_file << "," << std::endl;
                    }
                    js_file << "\t{ ";

                    js_file << "lat: "     << latitude                        << ", ";
                    js_file << "lng: "     << longitude                        << "";

                    js_file << " }";

                }
                js_file << std::endl << "];" << std::endl;

            }
            return 0;
        }


        std::string pathToOsrmFile = argc < (6 + arg_offset) ? "map_data/germany-latest.osrm" : argv[5 + arg_offset];

        double dampeningFactor = argc < (5 + arg_offset) ? 1.0 : std::stod(argv[4 + arg_offset]);


        // Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore
        EngineConfig config;

        config.storage_config = { pathToOsrmFile };
        config.use_shared_memory = false;

        // We support two routing speed up techniques:
        // - Contraction Hierarchies (CH): requires extract+contract pre-processing
        // - Multi-Level Dijkstra (MLD): requires extract+partition+customize pre-processing
        //
        // config.algorithm = EngineConfig::Algorithm::CH;
        config.algorithm = EngineConfig::Algorithm::MLD;

        // Routing machine with several services (such as Route, Table, Nearest, Trip, Match)
        const OSRM osrm{ config };



        // Response is in JSON format
        engine::api::ResultT result = json::Object();

        auto& json_result = result.get<json::Object>();

        //todo move out of scope uninitialized values
        const auto status = osrm.Table(params, result);

        if (status == Status::Error)
        {
            std::string code = json_result.values["code"].get<json::String>().value;
            std::string message = json_result.values["message"].get<json::String>().value;

            throw std::runtime_error("OSRM error: " + code + ". " + message);
        }

        const auto& durations_matrix = json_result.values["durations"].get<json::Array>();
        if (work_mode == WorkMode::result) {
            std::string resultInputFilename = argv[4];

            std::unordered_map<int, std::unordered_map<int, double> > drivingTimeFromTo;
            for (size_t indexFrom = 0; indexFrom < durations_matrix.values.size(); indexFrom++) {
                auto& durations_array = durations_matrix.values.at(indexFrom);
                auto& durations = durations_array.get<json::Array>();
                const int jobFromId = jobRowToId.at(indexFrom);
                for (size_t indexTo = 0; indexTo < durations.values.size(); indexTo++) {
                    const int jobToId = jobRowToId.at(indexTo);
                    auto duration_value = durations.values[indexTo];
                    auto duration = duration_value.get<json::Number>();
                    drivingTimeFromTo[jobFromId][jobToId] = duration.value;
                }
            }

            std::ifstream resultInputFile(resultInputFilename);
            if (!resultInputFile.is_open()) {
                throw std::runtime_error("error opening input file " + resultInputFilename);
            }

            //output results file
            std::string stopListLine;
            std::getline(resultInputFile, stopListLine);
            outputFile << stopListLine << '\n';

            {
                std::string score1Line;
                std::getline(resultInputFile, score1Line);
                outputFile << score1Line << '\n';
            }
                
            std::istringstream lineStream(stopListLine);
            std::vector<int> jobIds;
            std::copy(std::istream_iterator<int>(lineStream), std::istream_iterator<int>(), std::back_inserter(jobIds));

            double workDurationSum = 0;
            for (size_t i = 0; i < jobIds.size(); i++) {
                workDurationSum += workDurationMap.at(jobIds[i]);
            }
            outputFile << workDurationSum << '\n';

            double baseDrivingTimeSum = 0;
            for (size_t i = 0; i < jobIds.size() - 1; i++) {
                int fromId = jobIds[i];
                int toId = jobIds[i + 1];
                baseDrivingTimeSum += drivingTimeFromTo.at(fromId).at(toId);

            }
            const double drivingTimeSumMinutes = baseDrivingTimeSum * dampeningFactor / 60.0;
            outputFile << std::lround(drivingTimeSumMinutes) << '\n';

            const double totalTimeHours = (workDurationSum + drivingTimeSumMinutes)/60.0;
            outputFile << std::fixed << std::setprecision(1) << totalTimeHours << std::setprecision(0) << '\n';

            for (size_t i = 0; i < jobIds.size(); i++) {
                int drivingTimeFromPreviousJob = 0;
                if (i != 0) {
                    drivingTimeFromPreviousJob = std::lround(drivingTimeFromTo.at(jobIds[i - 1]).at(jobIds[i])* dampeningFactor / 60.0);
                }
                outputFile << jobIds[i] << ";" << drivingTimeFromPreviousJob << ";" << workDurationMap.at(jobIds[i]) << ";" << "\n";
            }
        }
        else {
            if (work_mode == WorkMode::workdrivesymc) {
                outputFile << "NAME: " << outputFilename << std::endl;
                outputFile << "COMMENT : based on input from "  << inputFilename << std::endl;
                outputFile << "TYPE : OP" << std::endl;
                outputFile << "DIMENSION : " << durations_matrix.values.size() <<  std::endl;
                outputFile << "COST_LIMIT : " << worktime_limit_in_minutes << std::endl;
                outputFile << "EDGE_WEIGHT_TYPE : EXPLICIT" << std::endl;
                outputFile << "EDGE_WEIGHT_FORMAT : LOWER_DIAG_ROW" << std::endl;
                outputFile << "NODE_COORD_TYPE : NO_COORDS" << std::endl;
                outputFile << "DISPLAY_DATA_TYPE : NO_DISPLAY" << std::endl;
                outputFile << "EDGE_WEIGHT_SECTION" << std::endl;
                for (size_t indexFrom = 0; indexFrom < durations_matrix.values.size(); indexFrom++) {
                    auto& durations_array = durations_matrix.values.at(indexFrom);
                    auto& durations = durations_array.get<json::Array>();
                    bool iterates_first_value = true;
//                    outputFile << "D,  ";

                    for (size_t indexTo = 0; indexTo <= indexFrom; indexTo++) {
                        auto duration_value = durations.values[indexTo];
                        auto duration = duration_value.get<json::Number>();
                        double durationInMinutes = duration.value * dampeningFactor / 60.0;
                        durationInMinutes += workDurationMap.at(jobRowToId.at(indexTo)) / 2.0;
                        durationInMinutes += workDurationMap.at(jobRowToId.at(indexFrom)) / 2.0;

                        if (!iterates_first_value) {
                            outputFile << " ";
                        }
                        outputFile << std::lround(durationInMinutes);
                        iterates_first_value = false;
                    }
                    outputFile << '\n';
                }
                outputFile << "NODE_SCORE_SECTION" << std::endl;
                for (size_t i = 0; i < durations_matrix.values.size(); i++) {
                    outputFile << i + 1 << " " << score1Map.at(jobRowToId.at(i)) << std::endl;
                }

                outputFile << "EOF" << std::endl;                
            }
            else {
                for (size_t indexFrom = 0; indexFrom < durations_matrix.values.size(); indexFrom++) {
                    auto& durations_array = durations_matrix.values.at(indexFrom);
                    auto& durations = durations_array.get<json::Array>();
                    bool iterates_first_value = true;
                    outputFile << "D,  ";

                    for (size_t indexTo = 0; indexTo < durations.values.size(); indexTo++) {
                        auto duration_value = durations.values[indexTo];
                        auto duration = duration_value.get<json::Number>();
                        double durationInMinutes = duration.value * dampeningFactor / 60.0;
                        if (work_mode == WorkMode::workdrive) {
                            durationInMinutes += workDurationMap.at(jobRowToId.at(indexTo));
                        }
                        if (!iterates_first_value) {
                            outputFile << ",\t";
                        }
                        outputFile << std::lround(durationInMinutes);
                        iterates_first_value = false;
                    }
                    outputFile << ',' << '\n';
                }
            }
        }
        return 0;
    }
    catch (std::exception& ex) {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
}
