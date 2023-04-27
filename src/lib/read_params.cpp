#include "readParams.hpp"


namespace {

static paramIMU createParamIMU(const std::string & file_path)
    {
        YAML::Node config = YAML::LoadFile(file_path);

        if (config["type"].as<std::string>() == "SensorOdom3d")
        {
            ParamSensors params = std::make_shared<ParamSensors>();

            params->k_disp_to_disp   = config["k_disp_to_disp"] .as<double>();
            params->k_disp_to_rot    = config["k_disp_to_rot"]  .as<double>();
            params->k_rot_to_rot     = config["k_rot_to_rot"]   .as<double>();
            params->min_disp_var     = config["min_disp_var"] .as<double>();
            params->min_rot_var      = config["min_rot_var"]  .as<double>();

            return params;
        }

        std::cout << "Bad configuration file. No sensor type found." << std::endl;
        return nullptr;


    }
}