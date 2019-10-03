#include "contiki.h"
#include "net/rime/rime.h"
#include "node-id.h"
#include "random.h"
#include <stdio.h>


/*---------------------------------------------------------------------------*/
PROCESS(sending_broadcast, "sending_broadcast");
AUTOSTART_PROCESSES(&sending_broadcast);
/*---------------------------------------------------------------------------*/

const unsigned char typeSink = 0x2<<6;   // Sink: 0x10

struct t_msg_signal {
    unsigned char type_metr;
};

struct t_message_unicast{
  unsigned char id_msg;
  int id_node;
  unsigned char data_bcns[3];
  char data_dst[3];
  unsigned long time;
};

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;


static void recv_uc(struct unicast_conn *c, const linkaddr_t *from) {

  printf("Mensaje UNICAST recibido en el SINK por parte del Nodo %d.%d:\n", from->u8[0], from->u8[1]);

  struct t_message_unicast *msg_content = packetbuf_dataptr();

  printf("ID Mensaje: %x - ID Nodo: %d - Tiempo: %ld\n", 
    msg_content->id_msg, 
    msg_content->id_node,
    msg_content->time
    );

  printf("Baliza %x con distancia %d.\n", 
    msg_content->data_bcns[0],
    msg_content->data_dst[0]
    );

  printf("Baliza %x con distancia %d.\n", 
    msg_content->data_bcns[1],
    msg_content->data_dst[1]
    );

  printf("Baliza %x con distancia %d.\n", 
    msg_content->data_bcns[2],
    msg_content->data_dst[2]
    );
}

static const struct unicast_callbacks unicast_callbacks = {recv_uc};
static struct unicast_conn uc;

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(sending_broadcast, ev, data){

  static struct etimer timer_sending;
  static struct t_msg_signal signal_msg;

  static int num_msg=0;

  PROCESS_EXITHANDLER(broadcast_close(&broadcast);) // al terminar el proceso, se deberá cerrar la conexión

  PROCESS_BEGIN();

  broadcast_open(&broadcast, 129, &broadcast_call); // abrir la conexión en el canal 128
  unicast_open(&uc, 146, &unicast_callbacks);

  random_init((int) node_id);
  etimer_set(&timer_sending, CLOCK_SECOND * 4 + random_rand() % (CLOCK_SECOND * 4));
  signal_msg.type_metr=typeSink;


  while(1) {

    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer_sending));
    // enviar mensaje
    packetbuf_copyfrom(&signal_msg, sizeof(struct t_msg_signal)); // copiar el cuerpo del mensaje en el buffer de salida
    broadcast_send(&broadcast);

    num_msg++;
    //printf("Mensaje broadcast '%d' enviado\n", num_msg);
    etimer_reset(&timer_sending);
  }

  PROCESS_END();
}
