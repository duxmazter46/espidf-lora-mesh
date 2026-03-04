idf_component_register(SRCS "root_cli.c"
                    REQUIRES mesh node_config nvs_flash root_log
                    INCLUDE_DIRS "include")
