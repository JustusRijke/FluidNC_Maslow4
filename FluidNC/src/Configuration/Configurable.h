// Copyright (c) 2021 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Generator.h"
#include "Parser.h"

#define PREFIXED_LOG_SUFFIX ": "
#define p_log_msg(x)                                                                                                                       \
    { log_msg(_config_path << PREFIXED_LOG_SUFFIX << x); }
#define p_log_debug(x)                                                                                                                     \
    { log_debug(_config_path << PREFIXED_LOG_SUFFIX << x); }
#define p_log_info(x)                                                                                                                      \
    { log_info(_config_path << PREFIXED_LOG_SUFFIX << x); }
#define p_log_warn(x)                                                                                                                      \
    { log_warn(_config_path << PREFIXED_LOG_SUFFIX << x); }
#define p_log_error(x)                                                                                                                     \
    { log_error(_config_path << PREFIXED_LOG_SUFFIX << x); }
#define p_log_config_error(x)                                                                                                              \
    { log_config_error(_config_path << PREFIXED_LOG_SUFFIX << x); }
#define p_log_fatal(x)                                                                                                                     \
    { log_fatal(_config_path << PREFIXED_LOG_SUFFIX << x); }

namespace Configuration {
    class HandlerBase;

    class Configurable {
        Configurable(const Configurable&) = delete;
        Configurable(Configurable&&)      = default;

        Configurable& operator=(const Configurable&) = delete;
        Configurable& operator=(Configurable&&) = default;

    public:
        Configurable() = default;

        virtual void validate() {};
        virtual void group(HandlerBase& handler) = 0;
        virtual void afterParse() {}
        // virtual const char* name() const = 0;

        virtual void               setConfigPath(const std::string& path) { _config_path = path; }
        virtual const std::string& getConfigPath() const { return _config_path; }

        virtual ~Configurable() {}

    protected:
        std::string _config_path;
    };
}
