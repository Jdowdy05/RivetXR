function(require_exact variable expected)
  if(NOT DEFINED ${variable})
    message(FATAL_ERROR "${variable} must be defined")
  endif()

  if(NOT "${${variable}}" STREQUAL "${expected}")
    message(FATAL_ERROR
      "${variable} must be '${expected}', but was '${${variable}}'")
  endif()
endfunction()

require_exact(NEWTON_VERSION "1.6.0.dev0")
require_exact(WARP_VERSION "1.18.0.dev2")
require_exact(NEWTON_COMMIT "d37f4d3d341ccce1e06a1dff21e9a054759b4855")
require_exact(WARP_COMMIT "d4de134b97b961f1a19bd76830e71ea7f9df2470")
