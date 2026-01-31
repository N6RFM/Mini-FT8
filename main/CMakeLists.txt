idf_component_register(
    SRCS "main.cpp"
         "autoseq.cpp"
         "stream_uac.cpp"
         "resample.cpp"
    INCLUDE_DIRS "."
    REQUIRES ui M5Unified M5GFX M5Cardputer ft8_lib usb_host_uac usb_host_cdc_acm fatfs sdmmc
    PRIV_REQUIRES spiffs
)
