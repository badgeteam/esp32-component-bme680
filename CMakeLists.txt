idf_component_register(
  SRCS "bme680.c"
  INCLUDE_DIRS include
  PRIV_REQUIRES
    bus-i2c
    driver
)
