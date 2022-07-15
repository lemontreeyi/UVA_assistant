typedef struct fifo
{
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  int front, rear;
} fifo;
void init_fifo(fifo* p);
bool is_empty_fifo(fifo* p);
bool is_full_fifo(fifo* p);
bool read_fifo(fifo* p, uint8_t* data);
bool write_fifo(fifo* p, uint8_t data);
fifo mav_fifo;

void init_fifo(fifo* p)
{
  p->front = p->rear = 0;
  memset(p->buf, 0, MAVLINK_MAX_PACKET_LEN);
}
bool is_empty_fifo(fifo* p)
{
  if(p->front == p->rear)
    return 1;
  else
    return 0;
}
bool is_full_fifo(fifo* p)
{
  if((p->rear - p->front + MAVLINK_MAX_PACKET_LEN) % MAVLINK_MAX_PACKET_LEN == MAVLINK_MAX_PACKET_LEN - 1)
    return 1;
  else
    return 0;
}
bool read_fifo(fifo* p, uint8_t* data)
{
  if(!is_empty_fifo(p))
  {
    p->buf[p->front] = *data;
    p->front = (p->front + 1) % MAVLINK_MAX_PACKET_LEN;
    return 1;
  }
  return 0;
}
bool write_fifo(fifo* p, uint8_t data)
{
  if(!is_full_fifo(p))
  {
    p->buf[p->rear] = data;
    p->rear = (p->rear + 1) % MAVLINK_MAX_PACKET_LEN;
    return 1;
  }
  return 0;
}
