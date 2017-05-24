struct SYSCFG {
  char          mqtt_host[20];
  char          mqtt_port[5];
  char          mqtt_user[20];
  char          mqtt_key[33];
  char          mqtt_sensor_name[20];
  char          mqtt_topic_temp[30];
  char          mqtt_topic_hum[30];
  char          mqtt_topic_pir[30];
  char          mqtt_topic_ldr[30];
  char          mqtt_interval[8];
} sysCfg __attribute__ ((packed));
