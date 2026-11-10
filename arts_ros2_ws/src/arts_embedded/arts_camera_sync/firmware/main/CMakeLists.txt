set(arts_sync_srcs
    "main.c"
    "sync_engine.c"
)
set(arts_sync_includes ".")
set(arts_sync_requires
    driver          # mcpwm (generator + capture), gpio, uart
    esp_rom         # esp_rom_gpio_connect_out_signal, for the trigger fan-out
    esp_timer
)

set(uros_component "${CMAKE_CURRENT_LIST_DIR}/../components/micro_ros_espidf_component")

# The micro-ROS dependency has to be conditional in BOTH directions. Listing it
# unconditionally breaks BLINK and SELFTEST, because `vcs import components` is a
# MICROROS-only step and naming an absent component in REQUIRES is a hard error — a fresh
# clone could not build the mode used to bring the hardware up. Omitting it in the MICROROS
# build is the failure of 2026-08-18: libmicroros builds, the generated header is written,
# and main.c still cannot find it, because without the REQUIRES entry NONE of the component's
# include directories reach main's compile line. That error names the header and helpfully
# prints the three places it exists, which sends you auditing include paths and the
# extra_packages symlink instead of looking at this list.
#
# THE CONDITION IS THE COMPONENT'S PRESENCE ON DISK, NOT `CONFIG_ARTS_SYNC_MODE_MICROROS`,
# and the obvious Kconfig version SILENTLY DOES NOTHING. ESP-IDF processes each component's
# CMakeLists.txt TWICE: an early-expansion pass that collects REQUIRES to build the dependency
# graph, then the real pass. Kconfig values are not loaded yet in the first one — verified here
# 2026-08-18 by printing the symbol, which came out '' then 'y'. So a config-gated REQUIRES is
# always evaluated false where it matters, and there is no error to notice: the graph is simply
# built without the edge. Existence is knowable in both passes.
if(EXISTS "${uros_component}/CMakeLists.txt")
    list(APPEND arts_sync_requires micro_ros_espidf_component)

    # THE UART TRANSPORT IS NOT PART OF THE COMPONENT. `esp32_serial_open/close/write/read`,
    # which main.c hands to `rmw_uros_set_custom_transport()`, live in
    # examples/int32_publisher_custom_transport/main/ — upstream's design is that every
    # application copies those two files into its own main/. So the component provides the
    # declaration of the transport HOOK (rmw_microros/custom_transport.h) and none of the
    # implementation, and the symptom of not knowing that is a missing
    # `esp32_serial_transport.h` immediately after the messages finally resolve.
    #
    # Compiled from the fetched checkout rather than copied in, because components/ is
    # gitignored precisely so no third-party source is vendored here, and the file is pinned
    # by commit in components.repos so the path cannot shift underneath us. Reaching into an
    # examples/ directory is admittedly unusual; the alternative is a vendored copy that
    # silently stops matching the pinned component.
    #
    # Unlike REQUIRES, the SOURCE is gated on the run mode. It reads CONFIG_MICROROS_UART_*,
    # which exist only in a MICROROS sdkconfig, so compiling it into SELFTEST or BLINK fails —
    # and once components/ has been fetched that is every switch back from MICROROS (found
    # 2026-10-28). SRCS matter only in the real pass, where Kconfig IS loaded, so the gate
    # that cannot work for REQUIRES works here.
    if(CONFIG_ARTS_SYNC_MODE_MICROROS)
        set(uros_transport "${uros_component}/examples/int32_publisher_custom_transport/main")
        if(NOT EXISTS "${uros_transport}/esp32_serial_transport.c")
            message(FATAL_ERROR
                "micro-ROS UART transport not found at ${uros_transport}. Upstream moved it; "
                "find esp32_serial_transport.c in the component and update this path.")
        endif()
        list(APPEND arts_sync_srcs "${uros_transport}/esp32_serial_transport.c")
        list(APPEND arts_sync_includes "${uros_transport}")
    endif()
endif()

idf_component_register(
  SRCS ${arts_sync_srcs}
  INCLUDE_DIRS ${arts_sync_includes}
  REQUIRES ${arts_sync_requires}
)
