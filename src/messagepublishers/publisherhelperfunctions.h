// Copyright 2021 BlueSpace.ai, Inc.
//
// Source code modified from the source provided by Xsens Technologies


#ifndef PUBLISHER_HELPER_FUNCTION_H
#define PUBLISHER_HELPER_FUNCTION_H

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

class PublisherHelperFunctions
{
public:
    PublisherHelperFunctions(/* args */);
    ~PublisherHelperFunctions();

    void variance_from_stddev_param(std::string param, double *variance_out, rclcpp::Node& node_handle)
    {
        std::vector<double> stddev;
        if (node_handle.get_parameter(param, stddev))
        {
            if (stddev.size() == 3)
            {
                auto squared = [](double x) { return x * x; };
                std::transform(stddev.begin(), stddev.end(), variance_out, squared);
            }
            else
            {
                RCLCPP_WARN(node_handle.get_logger(), "Wrong size of param: %s, must be of size 3", param.c_str());
            }
        }
        else
        {
            memset(variance_out, 0, 3 * sizeof(double));
        }
    }

};

PublisherHelperFunctions::PublisherHelperFunctions(/* args */)
{
}

PublisherHelperFunctions::~PublisherHelperFunctions()
{
}

#endif

