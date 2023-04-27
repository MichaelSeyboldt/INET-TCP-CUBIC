//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

import inet.common.INETDefs;
import inet.transportlayer.tcp.flavours.TcpBaseAlgState;
import inet.transportlayer.tcp.flavours.TcpSegmentTransmitInfoList;
//import inet.transportlayer.tcp.flavours.TcpTahoeRenoFamilyState;
namespace inet::tcp;


struct TcpCubicStateVariables extends TcpBaseAlgStateVariables
{
    @descriptor(readonly);
	
    TcpSegmentTransmitInfoList regions;

    uint32_t ssthresh = 4294967295; ///< slow start threshold

    ///cubic variables
    uint32_t c_w_last_max;
    simtime_t c_epoch_start = SIMTIME_ZERO;
    uint32_t c_origin_point; 
    simtime_t c_dMin = SIMTIME_MAX; 
    uint32_t c_ack_cnt;
    uint32_t c_w_tcp;
    simtime_t c_K;
    uint32_t c_w_max;
    uint32_t c_cwnd_cnt = 0;
    uint32_t c_cnt = 0;
    uint32_t c_target = 0;
    simtime_t c_t = SIMTIME_ZERO;
    simtime_t c_offs = SIMTIME_ZERO;
    
    //// parameters
    double c_litle_b;
    double c_big_C;
    bool c_tcp_friendliness = 1;
    bool c_fast_convergence = 1;
    
    
};


cplusplus(TcpCubicStateVariables) {{
  public:
    virtual std::string str() const override;
    virtual std::string detailedInfo() const override;
}}