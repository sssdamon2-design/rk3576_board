
#pragma once

#include <string>

struct InferenceResult
{
    bool success = false;
    std::string text;
    std::string error_message;
};
