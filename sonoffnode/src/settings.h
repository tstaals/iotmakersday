struct SYSCFG {
  char          mqtt_host[20];
  char          mqtt_port[5];
  char          mqtt_user[20];
  char          mqtt_key[33];
  char          mqtt_sensor_name[20];
  char          mqtt_topic_relay[30];
  char          mqtt_topic_relay_on[4];
  char          mqtt_topic_relay_off[4];
} sysCfg __attribute__ ((packed));
