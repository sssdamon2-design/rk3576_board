
#pragma once

#include <string>
#include "common/inference_result.h"

class ILLMBackend
{
public:
    virtual ~ILLMBackend() = default ;

    virtual InferenceResult infer(const std::string& question) = 0;

    virtual void shutdown()noexcept=0;



};

