
#pragma once

#include "ai/llm_backend.h"

class FakeLLMBackend : public ILLMBackend
{
public:
    InferenceResult infer(
        const std::string& question) override;

    void shutdown() noexcept override;
};
