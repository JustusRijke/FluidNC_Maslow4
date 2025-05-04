#pragma once
#include <string>
#include <stdexcept>
#include "../../Logging.h"

#define PREFIXED_LOG_SEPARATOR "->"
#define PREFIXED_LOG_SUFFIX ": "
#define p_log_msg(x) { log_msg(_fullPath << PREFIXED_LOG_SUFFIX << x); }
#define p_log_debug(x) { log_debug(_fullPath << PREFIXED_LOG_SUFFIX << x); }
#define p_log_info(x) { log_info(_fullPath << PREFIXED_LOG_SUFFIX << x); }
#define p_log_warn(x) { log_warn(_fullPath << PREFIXED_LOG_SUFFIX << x); }
#define p_log_error(x) { log_error(_fullPath << PREFIXED_LOG_SUFFIX << x); }
#define p_log_config_error(x) { log_config_error(_fullPath << PREFIXED_LOG_SUFFIX << x); }
#define p_log_fatal(x) { log_fatal(_fullPath << PREFIXED_LOG_SUFFIX << x); }


/// Mixin that, once initialized, holds a name and computes its full “path”
/// by joining parent paths with ':'. This allows for hierarchical name prefixes when logging.
class HierarchicalLog {

public:
    HierarchicalLog() = default;
    virtual ~HierarchicalLog() = default;

    std::string _name;
    std::string _fullPath;

    /// Call exactly once, after parent (if any) is already initialized.
    /// Throws if `name` is empty.
    void initName(std::string name, const HierarchicalLog* parent = nullptr) {
        if (name.empty())
            throw std::invalid_argument{"name must not be empty"};

        _name = std::move(name);
        if (parent && !parent->_fullPath.empty())
            _fullPath = parent->_fullPath + PREFIXED_LOG_SEPARATOR + _name;
        else
            _fullPath = _name;
    }

    /// Returns the last component.
    const std::string& name()     const noexcept { return _name; }
    /// Returns “parent:child:me”
    const std::string& fullPath() const noexcept { return _fullPath; }
};
