set(component_srcs "source/bee_Lena_r8.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver" "sht3x_sensor_" "cJson" "uart" "output"  "rs485"
                       REQUIRES "")