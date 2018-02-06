#include "Log.hpp"

namespace slam
{
    static std::shared_ptr<spdlog::logger> logger_(nullptr);
    
    std::shared_ptr<spdlog::logger> logger()
    {
        if(!logger_)
            logger_ = spdlog::stdout_logger_mt("console");

        return logger_;
    }
}
