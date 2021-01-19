/* ---------------------------- Data definitions ---------------------------- */

// Packet of data to send to the receiver
struct packet2Recv {
  uint8_t mode;
  int servoPulse;
  float current;
};
typedef struct packet2Recv packet2Recv;

// Packet of data to send to the controler
struct packet2Cont {
  uint32_t pingCounter;
};
typedef struct packet2Cont packet2Cont;

//
struct cruiseControl {
  bool updateSpeed;
  int servoPulse;
  bool halt;
};
typedef struct cruiseControl cruiseControl;
