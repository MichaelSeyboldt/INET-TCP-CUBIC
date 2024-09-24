/*
 * TcpCubic.cc
 *
 *  Created on: Sep 23, 2022
 *      Author: iangelis
 */
#include "inet/transportlayer/tcp/Tcp.h"
#include "inet/transportlayer/tcp/flavours/TcpCubic.h"
#include <algorithm> // min,max
#include <cmath>



namespace inet {
namespace tcp {


Register_Class(TcpCubic);

simsignal_t TcpCubic::WmaxSignal = cComponent::registerSignal("Wmax"); // will record the WmaX

std::string TcpCubicStateVariables::str() const
{
    std::stringstream out;
    out << TcpBaseAlgStateVariables::str();
    //out << " beta =" << m_beta;
    return out.str();
}

std::string TcpCubicStateVariables::detailedInfo() const
{
    std::stringstream out;
    out << TcpBaseAlgStateVariables::detailedInfo();
    out << "ssthresh = " << ssthresh << "\n";
    //out << "m_minRTT = " << m_minRTT << "\n";
    return out.str();
}

TcpCubic::TcpCubic()
: TcpBaseAlg(), state((TcpCubicStateVariables *&)TcpAlgorithm::state)
{
}

void TcpCubic::initialize()
{
    TcpBaseAlg::initialize();
    cubic_reset();
    state->c_litle_b = 0.2;
    state->c_big_C = 0.4;
    state->c_tcp_friendliness = 1;
    state->c_fast_convergence = 1;
    _abc = true;
}

void TcpCubic::recalculateSlowStartThreshold() 
{

    state->c_epoch_start = SIMTIME_ZERO;
    if((state->snd_cwnd < state->c_w_last_max) && state->c_fast_convergence)
    {
        state->c_w_last_max = state->snd_cwnd * ((2.0 - state->c_litle_b)/2.0) ;
    }
    else
    {
        state->c_w_last_max = state->snd_cwnd;
    }

    state->ssthresh = std::max( static_cast<uint32_t>(state->snd_cwnd * (1.0 - state->c_litle_b)),2 * state->snd_mss);
    conn->emit(WmaxSignal,state->c_w_last_max);
    conn->emit(ssthreshSignal, state->ssthresh);
}


void TcpCubic::processRexmitTimer(TcpEventCode& event)
{
    // deflate the congestion window to prevent the new ssthresh calculation to use the inflated value
    if(state->lossRecovery){
        state->snd_cwnd = state->ssthresh;
    } 

    TcpBaseAlg::processRexmitTimer(event);

    if (event == TCP_E_ABORT)
            return;

    // RFC 3782, page 6:
    // "6)  Retransmit timeouts:
    // After a retransmit timeout, record the highest sequence number
    // transmitted in the variable "recover" and exit the Fast Recovery
    // procedure if applicable."
    state->recover = (state->snd_max - 1);
    EV_INFO << "recover=" << state->recover << "\n";
    state->lossRecovery = false;
    state->firstPartialACK = false;
    EV_INFO << "Loss Recovery terminated.\n";

    // begin Slow Start (RFC 2581)
    recalculateSlowStartThreshold();
    cubic_reset();
    state->snd_cwnd = state->snd_mss;

    conn->emit(cwndSignal, state->snd_cwnd);

    EV_INFO << "Begin Slow Start: resetting cwnd to " << state->snd_cwnd
            << ", ssthresh=" << state->ssthresh << "\n";

    state->afterRto = true;
    state->inhibitRecovery  = true;

    conn->retransmitOneSegment(true);

}

void TcpCubic::cubic_reset()
{
    state->c_w_last_max = 0;
    state->c_epoch_start = SIMTIME_ZERO;
    state->c_origin_point =0;
    state->c_dMin = SIMTIME_ZERO;
    state->c_w_tcp = 0;
    state->c_ack_cnt =0;
    state->c_K = 0;
}


void TcpCubic::cubic_tcp_friendliness()
{
    
    double const_cal = ((3.0*state->c_litle_b)/(2.0 - state->c_litle_b));
    double mul = ((double)(state->c_ack_cnt*state->snd_mss))/((double)state->snd_cwnd);
    double resolt_mul = const_cal * mul;
    state->c_w_tcp = state->c_w_tcp + static_cast<uint32_t>(resolt_mul);
    state->c_ack_cnt = 0;
    if(state->c_w_tcp > state->snd_cwnd)
    {
        uint32_t max_cnt = (state->snd_cwnd / (state->c_w_tcp - state->snd_cwnd))/state->snd_mss;
        if( state->c_cnt > max_cnt)
        {
            state->c_cnt = max_cnt;
        }
    }
}


void TcpCubic::cubic_update()
{
    double K;
    simtime_t offs;
    uint32_t target;

    state->c_ack_cnt++; /* count the number of ACKed packets */


    if(state->c_epoch_start == SIMTIME_ZERO)
    {
        // Record the curent Time
        state->c_epoch_start = simTime();
        
        if((state->snd_cwnd) < (state->c_w_last_max))
        {

            double dividend = static_cast<double>(state->c_w_last_max - state->snd_cwnd);
            dividend = dividend/state->snd_mss; //convert to segments
            double cubic_root = dividend / state->c_big_C;
            K = std::cbrtl(cubic_root);
            state->c_K = SimTime(K); //convet to SimTime

            state->c_origin_point = state->c_w_last_max;
            
        }
        else
        {
            state->c_K = SIMTIME_ZERO;
            state->c_origin_point = state->snd_cwnd;
            
        }
        state->c_ack_cnt = 1;
        state->c_w_tcp = state->snd_cwnd;
    }

    simtime_t t = simTime() + state->c_dMin - state->c_epoch_start;
    state->c_t = t;
    double b_t = SIMTIME_DBL(t); //debug-info


    if(t < state->c_K) // |t - K|
    {
        offs = state->c_K - t;
    }
    else
    {
        offs = t - state->c_K;
    }
    state->c_offs = offs;//dubug perpuse

    // convert to double for pow 
    double d_delta = SIMTIME_DBL(offs);
    double cubic_result = std::pow(d_delta,3.0);


    //  origin_point + C*(t-K)^3
    uint32_t delta =  static_cast<uint32_t>(state->c_big_C * cubic_result);

    if(t < state->c_K)
    {
       target = (state->c_origin_point/state->snd_mss) - delta;
    }
    else
    {
        target = (state->c_origin_point/state->snd_mss) + delta;
    }
    state->c_target = target;// dubug perpuse


    if (target > (state->snd_cwnd/state->snd_mss))
    {
        state->c_cnt = (state->snd_cwnd/state->snd_mss) / (target - (state->snd_cwnd/state->snd_mss));
    }
    else{
        state->c_cnt = 100 * (state->snd_cwnd/state->snd_mss);
    }


    if(state->c_tcp_friendliness == true)
    {
        //achieve performance as TCP Reno Family
        cubic_tcp_friendliness();
    }
}


void TcpCubic::performSSCA()
{
    if (state->snd_cwnd <= state->ssthresh) {
        EV_DETAIL << "cwnd <= ssthresh: Slow Start: increasing cwnd by SMSS bytes to ";

        // perform Slow Start. RFC 2581: "During slow start, a TCP increments cwnd
        // by at most SMSS bytes for each ACK received that acknowledges new data."

        // Note: we could increase cwnd based on the number of bytes being
        // acknowledged by each arriving ACK, rather than by the number of ACKs
        // that arrive. This is called "Appropriate Byte Counting" (ABC) and is
        // described in RFC 3465 (experimental).
        //
    
        if(_abc){
                   int bytesAcked = std::min(state->snd_una - firstSeqAcked,2*state->snd_mss);
               state->snd_cwnd += bytesAcked;
        }else{
            state->snd_cwnd += state->snd_mss;
        }

        conn->emit(cwndSignal, state->snd_cwnd);
        EV_DETAIL << "cwnd=" << state->snd_cwnd << "\n";
    }
    else {
            // perform Congestion Avoidance (RFC 8312)

            cubic_update();
            if(state->c_cwnd_cnt > state->c_cnt)
            {
                state->snd_cwnd += state->snd_mss;
                conn->emit(cwndSignal, state->snd_cwnd);
                state->c_cwnd_cnt = 0;
            }else
            {
                state->c_cwnd_cnt++;
            }
            debugCubic();
    }
}


void TcpCubic::receivedDataAck(uint32_t firstSeqAcked)
{
    // TcpBaseAlg::receivedDataAck(firstSeqAcked) resets retxtimer every time, 
    // for partial acks we only want them reset for the first partial ack
    // so c + v for everything else in the parent function and add a check if the algorithm is in recovery

    // handling of retransmission timer: if the ACK is for the last segment sent
    // (no data in flight), cancel the timer, otherwise restart the timer
    // with the current RTO value.
    // but don't do this for partial acks
    //
    if (state->snd_una == state->snd_max) {
        if (rexmitTimer->isScheduled()) {
            EV_INFO << "ACK acks all outstanding segments, cancel REXMIT timer\n";
            cancelEvent(rexmitTimer);
        }
        else
            EV_INFO << "There were no outstanding segments, nothing new in this ACK.\n";
    }
    else if (!state->lossRecovery ){
        // only do this if we are not in loss recovery
        EV_INFO << "ACK acks some but not all outstanding segments ("
                << (state->snd_max - state->snd_una) << " bytes outstanding), but not in loss recovery"
                << "restarting REXMIT timer\n";
        cancelEvent(rexmitTimer);
        startRexmitTimer();
    }


    // strg c v
    if (!state->ts_enabled) {
        // if round-trip time measurement is running, check if rtseq has been acked
        if (state->rtseq_sendtime != 0 && seqLess(state->rtseq, state->snd_una)) {
            // print value
            EV_DETAIL << "Round-trip time measured on rtseq=" << state->rtseq << ": "
                      << floor((simTime() - state->rtseq_sendtime) * 1000 + 0.5) << "ms\n";

            rttMeasurementComplete(state->rtseq_sendtime, simTime()); // update RTT variables with new value

            // measurement finished
            state->rtseq_sendtime = 0;
        }
    }

    //
  
    //
    // handling of PERSIST timer:
    // If data sender received a zero-sized window, check retransmission timer.
    //  If retransmission timer is not scheduled, start PERSIST timer if not already
    //  running.
    //
    // If data sender received a non zero-sized window, check PERSIST timer.
    //  If PERSIST timer is scheduled, cancel PERSIST timer.
    //INFO (TcpConnection)i7refbottle.snd.tcp.conn-6: Duplicate ACK #3
    if (state->snd_wnd == 0) { // received zero-sized window?
        if (rexmitTimer->isScheduled()) {
            if (persistTimer->isScheduled()) {
                EV_INFO << "Received zero-sized window and REXMIT timer is running therefore PERSIST timer is canceled.\n";
                cancelEvent(persistTimer);
                state->persist_factor = 0;
            }
            else
                EV_INFO << "Received zero-sized window and REXMIT timer is running therefore PERSIST timer is not started.\n";
        }
        else {
            if (!persistTimer->isScheduled()) {
                EV_INFO << "Received zero-sized window therefore PERSIST timer is started.\n";
                conn->scheduleAfter(state->persist_timeout, persistTimer);
            }
            else
                EV_INFO << "Received zero-sized window and PERSIST timer is already running.\n";
        }
    }
    else { // received non zero-sized window?
        if (persistTimer->isScheduled()) {
            EV_INFO << "Received non zero-sized window therefore PERSIST timer is canceled.\n";
            cancelEvent(persistTimer);
            state->persist_factor = 0;
        }
    }
    // srg c v end


    const TcpSegmentTransmitInfoList::Item *found = state->regions.get(firstSeqAcked);

    /* INET do not use TCP timestamps it use a custom class 
    TcpSegmentTransmitInfoList to get this information*/
    if(found != nullptr)
    {
        simtime_t currentTime = simTime();
        simtime_t tSent = found->getFirstSentTime();

        simtime_t rtt = currentTime - tSent;
        //Find The RTT min
        if(state->c_dMin == SIMTIME_ZERO || state->c_dMin > rtt)
        {
            state->c_dMin = rtt;
        }
    }

    // reset inhibiting the next recovery pahse
    if(state->inhibitRecovery && !state->lossRecovery && seqGE(state->snd_una, state->recover + state->snd_mss)){
        state->inhibitRecovery = false;
    }

    if (state->lossRecovery) {
        if (seqGE(state->snd_una - 1, state->recover)) {
            // Exit Fast Recovery: deflating cwnd
            //
            // option (1): set cwnd to min (ssthresh, FlightSize + SMSS)
            //uint32_t flight_size = state->snd_max - state->snd_una;
            //state->snd_cwnd = std::min(state->ssthresh, flight_size + state->snd_mss);
            EV_INFO << "Fast Recovery - Full ACK received: Exit Fast Recovery, setting cwnd to " << state->snd_cwnd << "\n";
            // option (2): set cwnd to ssthresh
            state->snd_cwnd = state->ssthresh;

            //tcpEV << "Fast Recovery - Full ACK received: Exit Fast Recovery, setting cwnd to ssthresh=" << state->ssthresh << "\n";
            // TODO - If the second option (2) is selected, take measures to avoid a possible burst of data (maxburst)!
            conn->emit(cwndSignal, state->snd_cwnd);

            state->lossRecovery = false;
            state->firstPartialACK = false;
            EV_INFO << "Loss Recovery terminated.\n";
        }
        else {
            // RFC 3782, page 5:
            // "Partial acknowledgements:
            // If this ACK does *not* acknowledge all of the data up to and
            // including "recover", then this is a partial ACK.  In this case,
            // retransmit the first unacknowledged segment.  Deflate the
            // congestion window by the amount of new data acknowledged by the
            // cumulative acknowledgement field.  If the partial ACK
            // acknowledges at least one SMSS of new data, then add back SMSS
            // bytes to the congestion window.  As in Step 3, this artificially
            // inflates the congestion window in order to reflect the additional
            // segment that has left the network.  Send a new segment if
            // permitted by the new value of cwnd.  This "partial window
            // deflation" attempts to ensure that, when Fast Recovery eventually
            // ends, approximately ssthresh amount of data will be outstanding
            // in the network.  Do not exit the Fast Recovery procedure (i.e.,
            // if any duplicate ACKs subsequently arrive, execute Steps 3 and 4
            // above).
            //
            // For the first partial ACK that arrives during Fast Recovery, also
            // reset the retransmit timer.  Timer management is discussed in
            // more detail in Section 4."

            EV_INFO << "Fast Recovery - Partial ACK received: retransmitting the first unacknowledged segment\n";
            // retransmit first unacknowledged segment
            conn->retransmitOneSegment(false);

            // deflate cwnd by amount of new data acknowledged by cumulative acknowledgement field
            state->snd_cwnd -= state->snd_una - firstSeqAcked;

            conn->emit(cwndSignal, state->snd_cwnd);

            EV_INFO << "Fast Recovery: deflating cwnd by amount of new data acknowledged, new cwnd=" << state->snd_cwnd << "\n";

            // if the partial ACK acknowledges at least one SMSS of new data, then add back SMSS bytes to the cwnd
            if (state->snd_una - firstSeqAcked >= state->snd_mss) {
                state->snd_cwnd += state->snd_mss;

                conn->emit(cwndSignal, state->snd_cwnd);

                EV_DETAIL << "Fast Recovery: inflating cwnd by SMSS, new cwnd=" << state->snd_cwnd << "\n";
            }

            // try to send a new segment if permitted by the new value of cwnd
            sendData(false);

            // reset REXMIT timer for the first partial ACK that arrives during Fast Recovery
            if (state->lossRecovery) {
                if (!state->firstPartialACK) {
                    state->firstPartialACK = true;
                    EV_DETAIL << "First partial ACK arrived during recovery, restarting REXMIT timer.\n";
                    restartRexmitTimer();
                }
            }
        }
    }
    else
    {
        // Perform slow start and congestion avoidance.
        performSSCA();

        // RFC 3782, page 13:
        // "When not in Fast Recovery, the value of the state variable "recover"
        // should be pulled along with the value of the state variable for
        // acknowledgments (typically, "snd_una") so that, when large amounts of
        // data have been sent and acked, the sequence space does not wrap and
        // falsely indicate that Fast Recovery should not be entered (Section 3,
        // step 1, last paragraph)."
        //
        // this should not be done after RTOs
        if (!state->inhibitRecovery){
            state->recover = (state->snd_una - 2);
        }

    }

  state->regions.clearTo(state->snd_una);
  // ack and/or cwnd increase may have freed up some room in the window, try sending
  sendData(false);
}



void TcpCubic::receivedDuplicateAck()
{
    TcpBaseAlg::receivedDuplicateAck();
    //TCP Cubic use the do not change the Fast Retransmit, Fast Recovery
    //From TCP New Reno RFC 3782. Notes form paper
    //(A Deterministic Loss Model Based Analysis of CUBIC)
    if (state->dupacks == state->dupthresh) {

        //see new reno implematation
        if (!state->lossRecovery) {

            if (state->snd_una - 1 > state->recover) {

                recalculateSlowStartThreshold();

                state->recover = (state->snd_max - 1);
                state->firstPartialACK = false;
                state->lossRecovery = true;
                EV_INFO << " set recover=" << state->recover;

                state->snd_cwnd = state->ssthresh + 3*state->snd_mss;
                conn->emit(cwndSignal, state->snd_cwnd);


                EV_DETAIL << "Set cwnd=" << state->snd_cwnd << ", ssthresh=" << state->ssthresh << "\n";

                // Fast Retransmission: retransmit missing segment without waiting
                // for the REXMIT timer to expire
                conn->retransmitOneSegment(false);

                // Do not restart REXMIT timer.
                // Note: Restart of REXMIT timer on retransmission is not part of RFC 2581, however optional in RFC 3517 if sent during recovery.
                // Resetting the REXMIT timer is discussed in RFC 2582/3782 (NewReno) and RFC 2988.

                // try to transmit new segments (RFC 2581)
                sendData(false);
            }
            else {
                EV_INFO << "NewCubic on dupAcks == DUPTHRESH(=" << state->dupthresh << ": not invoking Fast Retransmit and Fast Recovery\n";

                // RFC 3782, page 4:
                // "1B) Not invoking Fast Retransmit:
                // Do not enter the Fast Retransmit and Fast Recovery procedure.  In
                // particular, do not change ssthresh, do not go to Step 2 to
                // retransmit the "lost" segment, and do not execute Step 3 upon
                // subsequent duplicate ACKs."
            }

        }
        EV_INFO << "Cubic on dupAcks == DUPTHRESH(=" << state->dupthresh << ": TCP is already in Fast Recovery procedure\n";


    }
    else if (state->dupacks > state->dupthresh)
    {
        if(state->lossRecovery)
        {
            // RFC 3782, page 4:
            // "3) Fast Recovery:
            // For each additional duplicate ACK received while in Fast
            // Recovery, increment cwnd by SMSS.  This artificially inflates the
            // congestion window in order to reflect the additional segment that
            // has left the network."
            // Cubic: like Reno meaby in need to chanhe in New reno for fast recovery
            state->snd_cwnd += state->snd_mss;
            conn->emit(cwndSignal, state->snd_cwnd);
            EV_DETAIL << "Cubic on dupAcks > DUPTHRESH(=" << state->dupthresh << ": Fast Recovery: inflating cwnd by SMSS, new cwnd=" << state->snd_cwnd << "\n";

            sendData(false);
        }
    }

}

void TcpCubic::dataSent(uint32_t fromseq)
{
    TcpBaseAlg::dataSent(fromseq);
    // save time when packet is sent
    // fromseq is the seq number of the 1st sent byte
    simtime_t sendtime = simTime();
    state->regions.clearTo(state->snd_una);
    state->regions.set(fromseq, state->snd_max, sendtime);
}


void TcpCubic::segmentRetransmitted(uint32_t fromseq, uint32_t toseq)
{
    TcpBaseAlg::segmentRetransmitted(fromseq, toseq);

    state->regions.set(fromseq, toseq, simTime());
}

void TcpCubic::debugCubic()
{
    EV_DETAIL << "============================================"<< endl
       <<"Time     " << simTime() << endl
       <<"Cwnd now " << state->snd_cwnd <<endl
       <<"ssthresh " << state->ssthresh <<endl
       <<"Min RTT  " << state->c_dMin << endl
       <<"target   " << state->c_target<< endl
       <<"K        " << state->c_K <<endl
       <<"t        " << state->c_t <<endl
       <<"offet    " << state->c_offs <<endl
       <<"c_cnt    " << state->c_cnt << endl
       <<"cwnd_cnt " << state->c_cwnd_cnt << endl
       << "============================================"
       << endl;
}

}
}
