#ifndef _UWB_HPP
#define _UWB_HPP

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <queue>
#include <vector>

#include "cx_uci.hpp"
#include "elog.h"

#define UWB_GENERAL_TIMEOUT_MS 5000

template <class Interface>
class CX310 {
   public:
    explicit CX310() : interface() {}
    explicit CX310(const Interface& i) : interface(i) { __init(); }

    ~CX310() {}

   private:
    constexpr static const char* TAG = "CX310";
    enum UwbsSTA : uint8_t { BOOT = 0, READY, ACTIVE, ERROR };

    Interface interface;
    UwbsSTA uwbs_sta = BOOT;
    bool init_success = false;

    UciCtrlPacket recv_packet;
    UciCMD uci_cmd;
    UciNTF uci_ntf;

    std::vector<uint8_t> rx_raw_buffer_vec;
    std::queue<uint8_t, std::deque<uint8_t>> rx_data_queue;
    std::queue<uint8_t, std::deque<uint8_t>> transparent_data;
    uint8_t _data;

    std::function<bool(const UciCtrlPacket&)> check_rsp = nullptr;
    std::function<bool()> cmd_packer = nullptr;

    /**
     * @brief 初始化
     */
   public:
    // 获取interface实例，用于中断处理
    Interface& get_interface() { return interface; }
    /**
     * @brief 初始化
     * @return 初始化成功返回true，失败返回false
     */
    bool reset(uint16_t timeout_ms = UWB_GENERAL_TIMEOUT_MS) {
        uwbs_sta = BOOT;
        uci_cmd.core_device_reset();
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_device_reset_rsp(rsp);
        };

        cmd_packer = [this]() { return uci_cmd.core_device_reset(); };

        if (__send_packet()) {
            uint32_t start_tick = interface.get_system_1ms_ticks();
            while (interface.get_system_1ms_ticks() - start_tick < timeout_ms) {
                update();
                if (uwbs_sta == READY) {
                    elog_v(TAG, "software reset successfully");
                    return true;
                }
            }
        }
        elog_e(TAG, "software reset fail");
        elog_e(TAG, "hardware reset start");

        interface.generate_reset_signal();
        interface.delay_ms(100);
        interface.turn_of_reset_signal();
        uint32_t start_tick = interface.get_system_1ms_ticks();
        while (interface.get_system_1ms_ticks() - start_tick < timeout_ms) {
            update();
            if (uwbs_sta == READY) {
                elog_v(TAG, "hardware reset successfully");
                return true;
            }
        }
        elog_e(TAG, "UWBS hardware reset failed");
        return false;
    }

    bool set_channel(uint8_t channel) {
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this, &channel]() {
            return uci_cmd.core_set_config(PARAM_CHANNEL_NUMBER_ID, 1,
                                           &channel);
        };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_set_config_rsp(rsp);
        };
        if (__send_packet()) {
            elog_v(TAG, "set channel %d", channel);
            return true;
        }
        elog_e(TAG, "set channel fail");
        return false;
    }

    bool get_channel(uint8_t& channel) {
        uint8_t param_id;
        uint8_t val_len;
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this]() {
            return uci_cmd.core_get_config(PARAM_CHANNEL_NUMBER_ID);
        };
        check_rsp = [this, &param_id, &val_len,
                     &channel](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_get_config_rsp(rsp, &param_id, &val_len,
                                                     &channel);
        };
        if (__send_packet()) {
            if (param_id != PARAM_CHANNEL_NUMBER_ID) {
                elog_e(TAG, "get config id %d", param_id);
                return false;
            } else {
                elog_v(TAG, "get channel %d", channel);
                return true;
            }
        }
        elog_e(TAG, "get channel fail");
        return false;
    }

    bool set_prf_mode(uint8_t prf_mode) {
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this, &prf_mode]() {
            return uci_cmd.core_set_config(PARAM_PRF_MODE_ID, 1, &prf_mode);
        };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_set_config_rsp(rsp);
        };
        if (__send_packet()) {
            elog_v(TAG, "set prf mode %d", prf_mode);
            return true;
        }
        elog_e(TAG, "set prf mode fail");
        return false;
    }

    bool get_prf_mode(uint8_t& prf_mode) {
        uint8_t param_id;
        uint8_t val_len;
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this]() {
            return uci_cmd.core_get_config(PARAM_PRF_MODE_ID);
        };
        check_rsp = [this, &param_id, &val_len,
                     &prf_mode](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_get_config_rsp(rsp, &param_id, &val_len,
                                                     &prf_mode);
        };
        if (__send_packet()) {
            if (param_id != PARAM_PRF_MODE_ID) {
                elog_e(TAG, "get config id %d", param_id);
                return false;
            } else {
                elog_v(TAG, "get prf mode %d", prf_mode);
                return true;
            }
        }
        elog_e(TAG, "get prf mode fail");
        return false;
    }

    bool set_preamble_length(uint8_t preamble_length) {
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this, &preamble_length]() {
            return uci_cmd.core_set_config(PARAM_PREAMBLE_LENGTH_ID, 1,
                                           &preamble_length);
        };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_set_config_rsp(rsp);
        };
        if (__send_packet()) {
            elog_v(TAG, "set preamble length %d", preamble_length);
            return true;
        }
        elog_e(TAG, "set preamble length fail");
        return false;
    }

    bool get_preamble_length(uint8_t& preamble_length) {
        uint8_t param_id;
        uint8_t val_len;
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this]() {
            return uci_cmd.core_get_config(PARAM_PREAMBLE_LENGTH_ID);
        };
        check_rsp = [this, &param_id, &val_len,
                     &preamble_length](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_get_config_rsp(rsp, &param_id, &val_len,
                                                     &preamble_length);
        };
        if (__send_packet()) {
            if (param_id != PARAM_PREAMBLE_LENGTH_ID) {
                elog_e(TAG, "get config id %d", param_id);
                return false;
            } else {
                elog_v(TAG, "get preamble length %d", preamble_length);
                return true;
            }
        }
        elog_e(TAG, "get preamble length fail");
        return false;
    }

    bool set_preamble_index(uint8_t preamble_index) {
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this, &preamble_index]() {
            return uci_cmd.core_set_config(PARAM_PREAMBLE_CODE_INDEX_ID, 1,
                                           &preamble_index);
        };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_set_config_rsp(rsp);
        };
        if (__send_packet()) {
            elog_v(TAG, "set preamble index %d", preamble_index);
            return true;
        }
        elog_e(TAG, "set preamble index fail");
        return false;
    }

    bool get_preamble_index(uint8_t& preamble_index) {
        uint8_t param_id;
        uint8_t val_len;
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this]() {
            return uci_cmd.core_get_config(PARAM_PREAMBLE_CODE_INDEX_ID);
        };
        check_rsp = [this, &param_id, &val_len,
                     &preamble_index](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_get_config_rsp(rsp, &param_id, &val_len,
                                                     &preamble_index);
        };
        if (__send_packet()) {
            if (param_id != PARAM_PREAMBLE_CODE_INDEX_ID) {
                elog_e(TAG, "get config id %d", param_id);
                return false;
            } else {
                elog_v(TAG, "get preamble index %d", preamble_index);
                return true;
            }
        }
        elog_e(TAG, "get preamble index fail");
        return false;
    }

    bool set_psdu_data_rate(uint8_t psdu_data_rate) {
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this, &psdu_data_rate]() {
            return uci_cmd.core_set_config(PARAM_PSDU_DATA_RATE_ID, 1,
                                           &psdu_data_rate);
        };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_set_config_rsp(rsp);
        };
        if (__send_packet()) {
            elog_v(TAG, "set psdu data rate %d", psdu_data_rate);
            return true;
        }
        elog_e(TAG, "set psdu data rate fail");
        return false;
    }

    bool get_psdu_data_rate(uint8_t& psdu_data_rate) {
        uint8_t param_id;
        uint8_t val_len;
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this]() {
            return uci_cmd.core_get_config(PARAM_PSDU_DATA_RATE_ID);
        };
        check_rsp = [this, &param_id, &val_len,
                     &psdu_data_rate](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_get_config_rsp(rsp, &param_id, &val_len,
                                                     &psdu_data_rate);
        };
        if (__send_packet()) {
            if (param_id != PARAM_PSDU_DATA_RATE_ID) {
                elog_e(TAG, "get config id %d", param_id);
                return false;
            } else {
                elog_v(TAG, "get psdu data rate %d", psdu_data_rate);
                return true;
            }
        }
        elog_e(TAG, "get psdu data rate fail");
        return false;
    }

    bool set_phr_mode(uint8_t phr_mode) {
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this, &phr_mode]() {
            return uci_cmd.core_set_config(PARAM_PHR_MODE_ID, 1, &phr_mode);
        };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_set_config_rsp(rsp);
        };
        if (__send_packet()) {
            elog_v(TAG, "set phr mode %d", phr_mode);
            return true;
        }
        elog_e(TAG, "set phr mode fail");
        return false;
    }
    bool get_phr_mode(uint8_t& phr_mode) {
        uint8_t param_id;
        uint8_t val_len;
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this]() {
            return uci_cmd.core_get_config(PARAM_PHR_MODE_ID);
        };
        check_rsp = [this, &param_id, &val_len,
                     &phr_mode](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_get_config_rsp(rsp, &param_id, &val_len,
                                                     &phr_mode);
        };
        if (__send_packet()) {
            if (param_id != PARAM_PHR_MODE_ID) {
                elog_e(TAG, "get config id %d", param_id);
                return false;
            } else {
                elog_v(TAG, "get phr mode %d", phr_mode);
                return true;
            }
        }
        elog_e(TAG, "get phr mode fail");
        return false;
    }
    bool set_sfd_id(uint8_t sfd_id) {
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this, &sfd_id]() {
            return uci_cmd.core_set_config(PARAM_SFD_ID_ID, 1, &sfd_id);
        };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_set_config_rsp(rsp);
        };
        if (__send_packet()) {
            elog_v(TAG, "set sfd id %d", sfd_id);
            return true;
        }
        elog_e(TAG, "set sfd id fail");
        return false;
    }

    bool get_sfd_id(uint8_t& sfd_id) {
        uint8_t param_id;
        uint8_t val_len;
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this]() {
            return uci_cmd.core_get_config(PARAM_SFD_ID_ID);
        };
        check_rsp = [this, &param_id, &val_len,
                     &sfd_id](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_get_config_rsp(rsp, &param_id, &val_len,
                                                     &sfd_id);
        };
        if (__send_packet()) {
            if (param_id != PARAM_SFD_ID_ID) {
                elog_e(TAG, "get config id %d", param_id);
                return false;
            } else {
                elog_v(TAG, "get sfd id %d", sfd_id);
                return true;
            }
        }
        elog_e(TAG, "get sfd id fail");
        return false;
    }

    bool set_tx_power(uint8_t tx_power) {
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this, &tx_power]() {
            return uci_cmd.core_set_config(PARAM_TX_POWER_ID, 1, &tx_power);
        };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_set_config_rsp(rsp);
        };
        if (__send_packet()) {
            elog_v(TAG, "set tx power %d", tx_power);
            return true;
        }
        elog_e(TAG, "set tx power fail");
        return false;
    }

    bool set_hprf() {
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this]() { return uci_cmd.cx_set_hprf(); };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_cx_set_hprf_rsp(rsp);
        };
        if (__send_packet()) {
            elog_v(TAG, "set hprf");
            return true;
        }
        elog_e(TAG, "set hprf fail");
        return false;
    }
    bool set_nooploop() {
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this]() { return uci_cmd.cx_nooploop(); };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_cx_nooploop_rsp(rsp);
        };
        if (__send_packet()) {
            elog_v(TAG, "set nooploop");
            return true;
        }
        elog_e(TAG, "set nooploop fail");
        return false;
    }

    bool get_config(uint8_t config_id, uint8_t* config_value, uint8_t length) {
        return false;
    }

    /**
     * @brief 获取设备信息
     */
    bool get_dev_info() {
        if (uwbs_sta != READY) {
            elog_e(TAG, "UWBS not ready");
            return false;
        }

        UciCMD::UWBDeviceInfo dev_info;
        cmd_packer = [this]() { return uci_cmd.core_get_device_info(); };
        check_rsp = [this, &dev_info](const UciCtrlPacket& rsp) {
            return uci_cmd.check_core_get_device_info_rsp(rsp, dev_info);
        };

        if (__send_packet()) {
            elog_v(TAG,
                   "uci generic version = 0x%.4X, mac version = 0x%.4X, "
                   "phy version = 0x%.4X , uci test version = 0x%.4X",
                   dev_info.uci_ver, dev_info.mac_ver, dev_info.phy_ver,
                   dev_info.uci_test_ver);

            // uint8_t vendor_len=;
            return true;
        }
        elog_e(TAG, "get device info fail");
        return false;
    }

    /**
     * @brief 数据透传
     * @param data 发送数据
     * @return 发送成功返回true，失败返回false
     */
    bool data_transmit(const std::vector<uint8_t>& data) {
        if (!__check_rdy()) {
            return false;
        }

        if (data.size() == 0) {
            return true;
        }

        cmd_packer = [this, &data]() { return uci_cmd.cx_app_data_tx(data); };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_cx_app_data_tx_rsp(rsp);
        };

        if (__send_packet()) {
            elog_i(TAG, "data transmit");
            return true;
        }
        elog_e(TAG, "data transmit fail");
        return false;
    }
    bool data_transmit_tx_test(std::vector<uint8_t> data, uint16_t pack_size,
                               uint16_t pack_num) {
        if (!__check_rdy()) {
            return false;
        }

        data.reserve(pack_size);
        for (uint16_t i = 0; i < pack_size; i++) {
            data.push_back(i % 0xff);
        }
#define _CNT 2000
        int delay_cnt = _CNT;
        for (uint16_t i = 0; i < pack_num; i++) {
            data[0] = i & 0xff;
            data[1] = i >> 8;
            data_transmit(data);

            while (delay_cnt--) {
                // __NOP();
            }
            delay_cnt = _CNT;
            // interface.delay_ms(delay_cnt);
        }
        return true;
    }

    bool data_transmit_rx_test(std::vector<uint8_t> data, uint16_t pack_size,
                               uint16_t pack_num) {
        if (!__check_rdy()) {
            return false;
        }
        uint16_t loss_pack = 0;
        uint32_t time_ms = 0;
        bool start_flag = false;
        uint16_t recv_cnt = 0;
        uint16_t pack_id = 0;
        uint32_t start_tick = 0;
        while (true) {
            if (get_recv_data(data)) {
                pack_id = data[0] | (data[1] << 8);
                if (!start_flag) {
                    if (pack_id == 0) {
                        start_flag = true;
                        recv_cnt = 1;
                        start_tick = interface.get_system_1ms_ticks();
                    }
                } else {
                    if (pack_id < recv_cnt) {
                        elog_e(TAG, "pack id = %u, recv_cnt = %u", pack_id,
                               recv_cnt);
                        return false;
                    }
                    if (pack_id != recv_cnt) {
                        loss_pack += pack_id - recv_cnt;
                        recv_cnt = pack_id;
                    }
                    recv_cnt++;
                    // elog_v("UWB: recv recv_cnt = %d", recv_cnt);
                    if (recv_cnt == pack_num) {
                        time_ms = interface.get_system_1ms_ticks() - start_tick;
                        elog_v(TAG,
                               "data transmit rx test: transmit pack = "
                               "%u, loss pack = %u, "
                               "time = %ums, transmit data = %uB",
                               pack_num, loss_pack, time_ms,
                               pack_num * pack_size);
                        return true;
                    }
                }
            }
        }
        return true;
    }

    /**
     * @brief 设置接收模式
     * @return 设置成功返回true，失败返回false
     */
    bool set_recv_mode() {
        if (!__check_rdy()) {
            return false;
        }

        cmd_packer = [this]() { return uci_cmd.cx_app_data_rx(); };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_cx_app_data_rx_rsp(rsp);
        };

        if (__send_packet()) {
            elog_i(TAG, "set recv mode");
            return true;
        }
        elog_e(TAG, "set recv mode fail");
        return false;
    }

    /**
     * @brief 获取透传数据
     * @param recv_data 接收数据
     * @return 获取成功返回true，失败返回false
     */
    bool get_recv_data(std::vector<uint8_t>& recv_data) {
        update();
        if (!__check_rdy()) {
            return false;
        }
        if (transparent_data.empty()) {
            return false;
        }
        recv_data.clear();
        recv_data.reserve(transparent_data.size());
        while (transparent_data.empty() == false) {
            recv_data.push_back(transparent_data.front());
            transparent_data.pop();
        }
        return true;
    }

    /**
     * @brief 停止接收
     * @return 停止成功返回true，失败返回false
     */
    bool stop_recv() {
        if (!__check_rdy()) {
            return false;
        }
        cmd_packer = [this]() { return uci_cmd.cx_app_data_stop_rx(); };
        check_rsp = [this](const UciCtrlPacket& rsp) {
            return uci_cmd.check_cx_app_data_stop_rx_rsp(rsp);
        };
        if (__send_packet()) {
            elog_v(TAG, "stop recv");
            return true;
        }
        elog_e(TAG, "stop recv fail");
        return false;
    }

    /**
     * @brief 更新，监听通知和更新状态机
     * @return 无
     */
    void update() {
        __listening_ntf();
        __uwbs_state_machine();
    }

    bool is_init_success() const { return init_success; }

    bool init() {
        __init();

        init_success &= set_hprf();
        init_success &=
            set_channel(PARAM_CHANNEL_NUMBER_9);    // channel 5 // channel 5
        init_success &= set_prf_mode(PARAM_PRF_NOMINAL_64_M);    // PRF mode 3
        init_success &= set_preamble_length(
            PARAM_PREAMBLE_LEN_BPRF_64);          // preamble length 1
        init_success &= set_preamble_index(9);    // preamble index 9
        init_success &=
            set_psdu_data_rate(PARAM_PSDU_DATA_RATE_7_8);    // PSDU data rate 4
        init_success &=
            set_phr_mode(PARAM_PHYDATARATE_DRHM_HR);    // PHR mode 4
        init_success &= set_sfd_id(2);                  // SFD ID 3 // SFD ID 3
        init_success &= set_tx_power(1);                // TX power 5
        return 0;
    }

   private:
    void __delay_ms(uint32_t ms) { interface.delay_ms(ms); }
    bool __check_rdy() {
        if (uwbs_sta == READY) {
            return true;
        }
        elog_e(TAG, "UWBS not ready");
        return false;
    }
    void __uwbs_state_machine() {
        switch (uwbs_sta) {
            case BOOT: {
                // elog_v("UWB: boot");
                break;
            }
            case READY: {
                // elog_v("UWB: ready");
                break;
            }
            case ACTIVE: {
                break;
            }
            case ERROR: {
                break;
            }
        }
    }

    void __load_recv_data() { interface.get_recv_data(rx_data_queue); }

    bool __rsp_process(uint32_t timeout_ms) {
        uint32_t start_tick = interface.get_system_1ms_ticks();
        while (interface.get_system_1ms_ticks() - start_tick < timeout_ms) {
            __load_recv_data();
            while (rx_data_queue.empty() == false) {
                _data = rx_data_queue.front();
                rx_data_queue.pop();

                if (recv_packet.flow_parse(_data)) {
                    //  Log.r(recv_packet.packet.data(),
                    //   recv_packet.packet.size());
                    if (recv_packet.mt == MT_RSP) {
                        // 接收到响应
                        return true;
                    } else if (recv_packet.mt == MT_NTF) {
                        // 接收到通知
                        __notify_process();
                    }
                }
            }
        }
        elog_e(TAG, "wait rsp timeout");
        return false;
    }

    void __listening_ntf() {
        __load_recv_data();
        while (rx_data_queue.empty() == false) {
            _data = rx_data_queue.front();
            rx_data_queue.pop();
            if (recv_packet.flow_parse(_data)) {
                if (recv_packet.mt == MT_NTF) {
                    __notify_process();
                } else {
                    elog_e(TAG, "unexpected rsp packet");
                }
            }
        }
    }

    void __notify_process() {
        if (recv_packet.gid == GID0x00) {
            switch (recv_packet.oid) {
                case CORE_DEVICE_STATUS_NTF: {
                    uint8_t sta = uci_ntf.parse_core_device_status_ntf(
                        recv_packet.packet);
                    if (sta == DEVICE_STATE_READY) {
                        if (uwbs_sta == BOOT) {
                            uwbs_sta = READY;
                            elog_v(TAG, "UWBS move to active state");
                        }
                    }
                    break;
                }
                default: {
                    elog_v(TAG, "undealed notify: gid=0x00, oid=0x%.2X",
                           recv_packet.oid);
                    break;
                }
            }
        }
        if (recv_packet.gid == GID0x03) {
            switch (recv_packet.oid) {
                case CX_APP_DATA_TX_NTF: {
                    if (uci_ntf.parse_cx_app_data_tx_ntf(recv_packet.packet) !=
                        STATUS_OK) {
                        elog_e(TAG, "parse data tx ntf fail");
                    }
                    break;
                }
                case CX_APP_DATA_RX_NTF: {
                    if (!uci_ntf.parse_cx_app_data_rx_ntf(recv_packet.packet)) {
                        elog_e(TAG, "parse data rx ntf fail");
                    }

                    if (recv_packet.packet.size() < 2) {
                        elog_e(TAG, "rx payload size is too small");
                        break;
                    }
                    for (auto it = recv_packet.packet.begin() + 2;
                         it != recv_packet.packet.end(); it++) {
                        transparent_data.push(*it);
                    }
                    // elog_v("UWB: data receive, size=%u",
                    //               rx_payload.size() - 2);
                    break;
                }
                default: {
                    elog_v(TAG, "undealed notify: gid=0x03, oid=0x%.2X",
                           recv_packet.oid);
                    break;
                }
            }
        }
    }

    void __init() {
        uint32_t start_tick;
        uwbs_sta = BOOT;
        init_success = false;
        // 芯片使能引脚初始化
        interface.chip_en_init();
        // 复位引脚初始化
        interface.reset_pin_init();
        // 使能芯片
        interface.chip_disable();
        __delay_ms(100);

        // 初始化通信端口
        interface.commuication_peripheral_init();

        interface.chip_enable();
        __delay_ms(200);

        // 产生复位信号
        // interface.generate_reset_signal();
        // __delay_ms(100);

        // // 关闭复位信号
        // interface.turn_of_reset_signal();
        // __delay_ms(2000);

        // 更新通知与状态机
        start_tick = interface.get_system_1ms_ticks();
        while (uwbs_sta != READY) {    // 检查是否就绪
            update();
            if (interface.get_system_1ms_ticks() - start_tick > 1000) {
                // 超时
                elog_e(TAG, "init wait ready timeout");
                break;
            }
        }
        if (uwbs_sta == READY) {
            if (reset(1000)) {
                init_success = true;
                elog_v(TAG, "UWBS init success");
                return;
            }
        }
        elog_e(TAG, "UWBS init fail");

        // 复位指令
        // __delay_ms(500);
        // reset(3000);
    }
    bool __send_packet() {
        if ((cmd_packer == nullptr) || (check_rsp == nullptr)) {
            return false;
        }
        bool send_flag = true;
        bool pack_all_payload = false;
        bool ret = false;
        uint8_t index = 0;
        while (1) {
            if (send_flag) {
                send_flag = false;
                pack_all_payload = cmd_packer();
                interface.send(uci_cmd.packet);
                // Log.r(uci_cmd.packet.data(), uci_cmd.packet.size());
            }
            if (__rsp_process(UWB_GENERAL_TIMEOUT_MS)) {
                if (check_rsp(recv_packet)) {
                    send_flag = true;
                    if (pack_all_payload) {
                        ret = true;
                        break;
                    }
                } else {
                    elog_e(TAG, "rsp check fail");
                    ret = false;
                    break;
                }
            } else {
                ret = false;
                break;
            }
        }
        uci_cmd.reset_packer();
        cmd_packer = nullptr;
        check_rsp = nullptr;
        return ret;
    }
};
#endif