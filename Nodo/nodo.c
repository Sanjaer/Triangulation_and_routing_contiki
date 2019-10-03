#include "contiki.h"
#include "net/rime/rime.h"
#include "dev/leds.h"
#include "random.h"
#include "node-id.h"

#include <stdio.h>
#include <stdlib.h>

#include "powertrace.h"

/*-------------------------------------------------------*/
PROCESS(nodo, "Nodo");
AUTOSTART_PROCESSES(&nodo);
/*-------------------------------------------------------*/

int gNodeID=-1;                 // To store the Node Id

const unsigned char typeBcon = 0x1<<6;   // Beacon: 0x01
const unsigned char typeSink = 0x2<<6;   // Sink: 0x10
const unsigned char typeNode = 0x3<<6;   // Node: 0x11

const char type_mask = 0xC0;    // To extract the type 

// Message for the broadcast communication
struct t_message_broadcast {
    unsigned char type_metr;
};

// Message for the unicast transmission
struct t_message_unicast{
  unsigned char id_msg;
  int id_node;
  unsigned char data_bcns[3];
  char data_dst[3];
  unsigned long time;
};

// struct type used for storing the id of the reference node
// to send the own beacons info and routed data via unicast
struct t_ref_node{
  unsigned char id;         // Stores node/sink id (from->u8[0])
  unsigned char type_metr;  // Stores type and metric
};
typedef struct t_ref_node ref_reg;

ref_reg working_reg;
ref_reg backup_reg;

/*---------------------------------------------------------------------------------------------*/

struct t_message_unicast msg_toBridge;
linkaddr_t addr_toBridge;
static struct unicast_conn uc;

/*---------------------------------------------------------------------------------------------*/

static unsigned short num_reg;            // Used as pointer to the first element of the triad of each register
static unsigned char beaconsListIds[60];  // Stores IDs of the Beacons
static signed char beaconsListDist[60];   // Stores the Distances of the Beacons
static unsigned long rawtime[20];         // Stores the time of consolidation of each triad

/*---------------------------------------------------------------------------------------------*/

static void recv_uc(struct unicast_conn *c, const linkaddr_t *from) {

 //printf("unicast received from %d.%d\n", from->u8[0], from->u8[1]);
  struct t_message_unicast *msg_content = packetbuf_dataptr();

  msg_toBridge.id_msg = msg_content->id_msg;
  msg_toBridge.id_node = msg_content->id_node;
  msg_toBridge.data_bcns[0] = msg_content->data_bcns[0];
  msg_toBridge.data_bcns[1] = msg_content->data_bcns[1];
  msg_toBridge.data_bcns[2] = msg_content->data_bcns[2];
  msg_toBridge.data_dst[0] = msg_content->data_dst[0];
  msg_toBridge.data_dst[1] = msg_content->data_dst[1];
  msg_toBridge.data_dst[2] = msg_content->data_dst[2];
  msg_toBridge.time = msg_content->time;

  packetbuf_copyfrom(&msg_toBridge, sizeof(struct t_message_unicast));
  addr_toBridge.u8[0] = working_reg.id; // establish destinatary direction
  addr_toBridge.u8[1] = 0;
  unicast_send(&uc, &addr_toBridge);

  printf("MSG UNICAST from NODO %d.%d resent to NODO %d.\n",
    from->u8[0], from->u8[1], working_reg.id);


}


void sent_uc(struct unicast_conn *c, int status, int num_tx){

  const linkaddr_t *dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);

  if(linkaddr_cmp(dest, &linkaddr_null)) {
    return;
  }

  printf("MSG UNICAST sent to NODO %d.%d ( Status: %d - Num_tx: %d).\n",
    dest->u8[0], dest->u8[1], status, num_tx);
}

/* Compose message */
void compose_msg(struct t_message_unicast* msg_content, unsigned short beaconIterator){

  msg_content->id_msg = typeNode;
  msg_content->id_node = gNodeID;
  msg_content->data_bcns[0] = beaconsListIds[beaconIterator + 0];
  msg_content->data_bcns[1] = beaconsListIds[beaconIterator + 1];
  msg_content->data_bcns[2] = beaconsListIds[beaconIterator + 2];
  msg_content->data_dst[0] = beaconsListDist[beaconIterator + 0];
  msg_content->data_dst[1] = beaconsListDist[beaconIterator + 1];
  msg_content->data_dst[2] = beaconsListDist[beaconIterator + 2];
  msg_content->time = rawtime[beaconIterator];

  /*printf("id_msg: %x, id_node: %d, data_bcns[0]: %x, data_dst[0]: %d, time: %ld\n", 
    msg_content->id_msg, 
    msg_content->id_node,
    msg_content->data_bcns[beaconIterator],
    msg_content->data_dst[beaconIterator],
    msg_content->time
    );*/

}

/*---------------------------------------------------------------------------------------------*/

/* Algorithm to update the register */
void fill_reg(unsigned char id, signed char dist){

  // Check if already inserted
  if (beaconsListIds[num_reg] == id){

    beaconsListDist[num_reg] = dist;
   // printf("Updated %d, %d\n", id, dist);

  } else if (beaconsListIds[num_reg+1] == id){
    
    beaconsListDist[num_reg+1] = dist;
   // printf("Updated +1 %d, %d\n", id, dist);
    
  } else if (beaconsListIds[num_reg+2] == id){
    
    beaconsListDist[num_reg+2] = dist;
   // printf("Updated +2 %d, %d\n", id, dist);
    
  } else if (beaconsListDist[num_reg] < dist){      // Better than 1º
   // printf("Substituted %d, %d\n", id, dist);

    beaconsListIds[num_reg+2] = beaconsListIds[num_reg+1];
    beaconsListDist[num_reg+2] = beaconsListDist[num_reg+1];
    beaconsListIds[num_reg+1] = beaconsListIds[num_reg];
    beaconsListDist[num_reg+1] = beaconsListDist[num_reg];
    beaconsListIds[num_reg] = id;
    beaconsListDist[num_reg] = dist;

  } else if (beaconsListDist[num_reg+1] < dist){    // Better than 2º
   // printf("Substituted +1 %d, %d\n", id, dist);

    beaconsListIds[num_reg+2] = beaconsListIds[num_reg+1];
    beaconsListDist[num_reg+2] = beaconsListDist[num_reg+1];
    beaconsListIds[num_reg+1] = id;
    beaconsListDist[num_reg+1] = dist;

  } else if (beaconsListDist[num_reg+2] < dist){    // Better than 3º
   // printf("Substituted +2 %d, %d\n", id, dist);

    beaconsListIds[num_reg] = id;
    beaconsListDist[num_reg] = dist;

  }

/*printf("beaconsListIds[num_reg]=%d\n"
"beaconsListIds[num_reg+1]=%d\n"
"beaconsListIds[num_reg+2]=%d\n"
"beaconsListDist[num_reg]=%d\n"
"beaconsListDist[num_reg+1]=%d\n"
"beaconsListDist[num_reg+2]=%d\n",
beaconsListIds[num_reg],
beaconsListIds[num_reg+1],
beaconsListIds[num_reg+2],
beaconsListDist[num_reg],
beaconsListDist[num_reg+1],
beaconsListDist[num_reg+2]);*/


}

void enrut_recv(unsigned char id_recv, unsigned char type_metr_recv) {

  // If Sink, always recorded
  if((type_metr_recv & type_mask) == typeSink){

 //printf("Update with sink\n");
    
    backup_reg.id = id_recv;
    backup_reg.type_metr = typeSink;
 
  } else if (type_metr_recv < backup_reg.type_metr){ // If distance is better (negative value, the closest to 0 the better)
 //printf("Update with node=%d, distance=%x\n", id_recv, type_metr_recv);

    backup_reg.id = id_recv;    
    backup_reg.type_metr = type_metr_recv;

  }
}


void update_working_reg(){

  static short count = 0; // Alive counter

 //printf("Before update:wr.id=%d, wr.tm=%x\n", working_reg.id, working_reg.type_metr);

  if ((working_reg.type_metr >= backup_reg.type_metr) || count == 3){
    working_reg.id = backup_reg.id; // Update working register with latest data
    working_reg.type_metr = backup_reg.type_metr;
    count = 0;
  } else{
    count++;
  }

  backup_reg.id = 0x00;             // Reset ID
  backup_reg.type_metr = 0xFF;      // Max num so it always gets overwritten
 //printf("After update:wr.id=%d, wr.tm=%x\n", working_reg.id, working_reg.type_metr);

}


static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {

  struct t_message_broadcast *msg_content = packetbuf_dataptr();

  signed char dist = (signed char)packetbuf_attr(PACKETBUF_ATTR_RSSI);

 //printf("MSG %x received from %x, rssi = %d\n", msg_content->type_metr, from->u8[0], dist);

  // If the message received is either a Node or a Sink means it is routing data
  if (msg_content->type_metr != typeBcon){

    enrut_recv(from->u8[0], msg_content->type_metr);

  } else {  // beacons data

    fill_reg(from->u8[0],dist);

  }

}

/*---------------------------------------------------------------------------------------------*/


static const struct unicast_callbacks unicast_callbacks = {recv_uc, sent_uc};
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;

/*------------------------------------------------------*/

PROCESS_THREAD(nodo, ev, data){

  static struct etimer timer_sending;
  static struct t_message_broadcast signal_msg;
  static struct t_message_unicast msg_content;
  linkaddr_t addr;
  unsigned short it;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

  PROCESS_BEGIN();
  
  // Initialize variables
  gNodeID = (int) node_id;
  working_reg.id = gNodeID;
  working_reg.type_metr = 0xFF;
  backup_reg.id = 0;
  backup_reg.type_metr = 0xFF;

  broadcast_open(&broadcast, 129, &broadcast_call); // Open conn
  unicast_open(&uc, 146, &unicast_callbacks);
  random_init((int) node_id);                       // Node ID used as seed for random wake-time
  etimer_set(&timer_sending, CLOCK_SECOND * 4 + random_rand() % (CLOCK_SECOND * 4));
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_sending));
  etimer_set(&timer_sending, CLOCK_SECOND * 10);    // 1 minute

  num_reg= 0;
  int i;
  for (i = 0; i < 60; i++){
    beaconsListDist[i] = -100;
  }

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_sending));

    update_working_reg();

   // printf("Awake %d, %ld, %d\n", gNodeID, rawtime[num_reg/3-1], num_reg);

    rawtime[num_reg/3] = clock_seconds();
    num_reg+=3;

    if(num_reg > 60) {
      //printf("num_reg = 60\n");
      
      
      for(it=0; it<20; it++) {

        compose_msg(&msg_content, it*3);
        // copy msg to output buffer
        packetbuf_copyfrom(&msg_content, sizeof(struct t_message_unicast));
        addr.u8[0] = working_reg.id;
        addr.u8[1] = 0;
        unicast_send(&uc, &addr);
      
      }
      
      num_reg = 0;
    } 
    

    /* Send MSG with routing data */
    if (working_reg.id != 0x00){

        signal_msg.type_metr = typeNode | (working_reg.type_metr + 0x01);
        packetbuf_copyfrom(&signal_msg, sizeof(struct t_message_broadcast));
        broadcast_send(&broadcast);

    }

    etimer_reset(&timer_sending);

  }

  PROCESS_END();

}

