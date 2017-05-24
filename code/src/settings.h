struct SYSCFG {
  char          mqtt_host[33];
  char          mqtt_port[5];
  char          mqtt_user[20];
  char          mqtt_key[33];
  char          mqtt_sensor_name[15];
  char          mqtt_topic_temp[33];
  char          mqtt_topic_hum[33];
  char          mqtt_topic_pir[33];
  char          mqtt_topic_ldr[33];
  char          mqtt_interval[8];
} sysCfg __attribute__ ((packed));
