#ifndef LegoPfReceiver_h
#define LegoPfReceiver_h

#include <WProgram.h>

//tijd in us van de bits, minimaal, maximaal.
#define s_min 947
#define s_max 1579
#define l_min 316
#define l_max 526
#define h_min 526
#define h_max 947

#define IS_0(t) ((t >= l_min) && (t < l_max))
#define IS_1(t) ((t >= h_min) && (t < h_max))
#define IS_S(t) ((t >= s_min) && (t < s_max))

#define CH1 0
#define CH2 1
#define CH3 2
#define CH4 3
#define CH5 4
#define CH6 5
#define CH7 6
#define CH8 7

class LegoPfReceiver
{
  private:
    uint32_t elapsedSince(uint32_t since, uint32_t now);
    uint32_t elapsedSince(uint32_t since);
    
    
  public:
    void SetInterrupt(int INT);
    boolean irRecv();
    void irRecvSignal();
    
    // PowerFunctions varibels
    uint8_t Nibble1;
    uint8_t Nibble2; 
    uint8_t Nibble3;   
};

extern LegoPfReceiver LEGOPFRECEIVER;

#endif