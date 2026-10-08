#ifndef __VALHALLA_EXCEPTIONS_H__
#define __VALHALLA_EXCEPTIONS_H__

#include <cstdint>
#include <stdexcept>
#include <string>

namespace valhalla {
class Api;

/**
 * Project specific error messages and codes that can be converted to http responses
 */
struct valhalla_exception_t : public std::runtime_error {
  /**
   * Constructs the exception by looking up predefined ones by their codes. If unsuccessful the code
   * will be 0
   * @param code   the code to look up
   * @param extra  an extra string to append to the codes existing method
   */
  valhalla_exception_t(unsigned code, const std::string& extra = "");
  /**
   * Patch 0050: as above, naming the request locations the error belongs to.
   * @param code               the code to look up
   * @param location_index     original index of the location (171) or of a failed pair's origin
   *                           (442, 447); -1 for none
   * @param destination_index  original index of a failed pair's destination; -1 for none
   * @param pruned             the failed pair's search ran with pruned hierarchy limits
   *                           although the costing disabled pruning (442, 447)
   */
  valhalla_exception_t(unsigned code,
                       int64_t location_index,
                       int64_t destination_index,
                       bool pruned = false);
  valhalla_exception_t(unsigned code,
                       const std::string& message,
                       unsigned http_code,
                       const std::string& http_message,
                       const std::string& osrm_error,
                       const std::string& statsd_key = "")
      : std::runtime_error(""), code(code), message(message), http_code(http_code),
        http_message(http_message), osrm_error(osrm_error), statsd_key(statsd_key) {
  }
  const char* what() const noexcept override {
    return message.c_str();
  }
  unsigned code;
  std::string message;
  unsigned http_code;
  std::string http_message;
  std::string osrm_error;
  std::string statsd_key;
  // Patch 0050: the original request index of the location an error belongs
  // to (171: the location itself; 442 and 447: the failed pair's origin) and of
  // the failed pair's destination; -1 when the error names no location.
  int64_t location_index = -1;
  int64_t destination_index = -1;
  // Patch 0050: the failed pair's search ran pruned although the costing
  // disables pruning (prune_hierarchy from the caller, or loki's per-pair
  // distance culling): a 442 there is no proof that no road exists.
  bool pruned = false;
};

/**
 * Adds a warning to the request PBF object.
 *
 * @param api   the full request
 * @param code  the warning code
 * @param extra an optional string to append to the hard-coded warning message
 */
void add_warning(valhalla::Api& api, unsigned code, const std::string& extra = "");
} // namespace valhalla

#endif //__VALHALLA_EXCEPTIONS_H__
