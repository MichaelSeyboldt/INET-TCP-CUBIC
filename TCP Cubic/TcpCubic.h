/*
 * TcpCubic.h
 *
 *  Created on: Sep 23, 2022
 *      Author: iangelis
 */

#ifndef INET_TRANSPORTLAYER_TCP_FLAVOURS_TCPCUBIC_H_
#define INET_TRANSPORTLAYER_TCP_FLAVOURS_TCPCUBIC_H_



#include "inet/transportlayer/tcp/flavours/TcpBaseAlg.h"
#include "inet/transportlayer/tcp/flavours/TcpCubicState_m.h"

namespace inet{
namespace tcp{

class INET_API TcpCubic : public TcpBaseAlg
{

    protected:

        static simsignal_t WmaxSignal; // will record the estimated pakages in the in the buffer (currently only with TcpVeno)

        TcpCubicStateVariables *& state; // alias to TCLAlgorithm's 'state'

        bool _abc; // use apropriate byte counting in the slow start phase

        virtual TcpStateVariables *createStateVariables() override
        {
            return new TcpCubicStateVariables();
        }

        /** Redefine what should happen on retransmission */
        virtual void processRexmitTimer(TcpEventCode& event) override;

        /** Utility function to recalculate ssthresh */
        virtual void recalculateSlowStartThreshold();

        virtual void initialize() override;

        void performSSCA();

        void cubic_reset();

        void cubic_tcp_friendliness();

        void cubic_update();

        void debugCubic();

    public:
        /** Ctor */
        TcpCubic();

        /** Redefine what should happen when data got acked, to add congestion window management */
        virtual void receivedDataAck(uint32_t firstSeqAcked) override;

        /** Redefine what should happen when dupAck was received, to add congestion window management */
        virtual void receivedDuplicateAck() override;

        /** Called after we send data */
        virtual void dataSent(uint32_t fromseq) override;

        virtual void  segmentRetransmitted(uint32_t fromseq, uint32_t toseq) override;

};


}
}


#endif /* INET_TRANSPORTLAYER_TCP_FLAVOURS_TCPCUBIC_H_ */
