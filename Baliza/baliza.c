#include "contiki.h"
#include "net/rime/rime.h"
#include "node-id.h"
#include "random.h"
#include <stdio.h>


/*---------------------------------------------------------------------------*/
PROCESS(sending_broadcast, "sending_broadcast");
AUTOSTART_PROCESSES(&sending_broadcast);
/*---------------------------------------------------------------------------*/

const unsigned char typeBcon = 0x1<<6;   // Beacon: 0x01

struct t_message_content {
    unsigned char type_metr;
};

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(sending_broadcast, ev, data){

  static struct etimer timer_sending;
  static struct t_message_content signal_msg;
  // static int num_msg;                               // To keep trak of the number of msgs sent

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);) // al terminar el proceso, se deberá cerrar la conexión

  PROCESS_BEGIN();                           

  broadcast_open(&broadcast, 129, &broadcast_call); // Open conn
  random_init((int) node_id);                       // Node ID used as seed for random wake-time
  etimer_set(&timer_sending, CLOCK_SECOND * 4 + random_rand() % (CLOCK_SECOND * 4));
  signal_msg.type_metr=typeBcon;                    // As content always sends the Type Beacon

  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_sending));
    // Send msg
    packetbuf_copyfrom(&signal_msg, sizeof(struct t_message_content)); // copiar el cuerpo del mensaje en el buffer de salida
    broadcast_send(&broadcast);
    //printf("Broadcast '%d' sent\n", num_msg);
    etimer_reset(&timer_sending);
  }

  PROCESS_END();
}
