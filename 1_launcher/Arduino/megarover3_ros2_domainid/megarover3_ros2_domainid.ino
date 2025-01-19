/**
 * スケッチ名：megarover3_ros2
 * バージョン：30.5.0
 * 概要：
 * 　メガローバーVer.3.0をROS 2で制御する際に汎用的に使用可能なスケッチです。
 * 　パッド->ROS(USBシリアル) or ROS(Wi-Fi)　の優先順位で操作コマンドを受理します。
 * 　
 *   ROS使用時にROSとの接続がない場合、自動的に停止します。ただし、付属ゲームパッドによる操作は有効です。
 * 
 *   VS-C3対応機体ではROVER_TYPE を MEGAROVER_3_F40A_C3 で定義してください。
 *   XBOXコントローラー対応機体では、ROVER_TYPE を ROVER_TYPE MEGAROVER_3_XBOX で定義してください。
 * 
 */
#define ROVER_TYPE MEGAROVER_3_F40A_C3  //VS-C3を使用するVS-WRC058cの場合はこちら
//#define ROVER_TYPE MEGAROVER_3_XBOX   //XBOXコントローラを使用するVS-WRC058の場合はこちら

/*******************************************************************
 * 本製品はROSから制御することが可能です。
 * WiFi接続と有線シリアル接続を選ぶことができます。両方同時には使えません。
 * WRC_ROS_SERIAL_MODEに使用する規格を定義してください。
 * 
 * 〇WiFi接続時
 * WRC_ROS_SERIAL_MODE を MODE_WIFI で定義してください。
 * 
 * 〇有線シリアル使用時
 * WRC_ROS_SERIAL_MODE を MODE_SERIAL で定義してください。
 * 
 * 〇ROSを使用しないとき
 * WRC_ROS_SERIAL_MODE を MODE_OFF で定義してください。
 */
#define MODE_OFF 0
#define MODE_WIFI 1
#define MODE_SERIAL 2
#define WRC_ROS_SERIAL_MODE MODE_SERIAL

/*******************************************************************
 * ウォッチドッグタイマに関する設定
 * 何らかの要因により外部機器との通信が停止した場合、WDTIMEに設定した時間[ms]後にモータを停止します。
 * 0を設定するとESP32によるタイマのクリアは無効化されますが、
 * 外部からメモリマップのMU16_WDTに値をセットすることでクリアすることができます。
 */
#define WDTIME 1000

#include <vs_wrc058_megarover.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int16_multi_array.h>
#include <micro_ros_utilities/type_utilities.h>

#include <geometry_msgs/msg/twist.h>

#include <WiFi.h>
#include <Arduino.h>
#include <NimBLEDevice.h>


/*******************************************
 * プロトタイプ宣言
 */

void LED(int cmd);


/*******************************************
 * WiFi関連の設定
 */
//char* ssid = "";      //WiFi使用時に接続するアクセスポイントのSSIDを設定してください ※2.4GHzのみ対応
//char* password = "";  //WiFi使用時に接続するSSIDのパスワードを設定してください

/*******************************************
 * ROS接続設定
 */
//char* ROSserverIP = "192.168.1.1";  //micro-ros Agentを起動したデバイスのIPアドレスを設定してください。
//int16_t serverPort = 8888;         //通信ポートを設定してください。

/*******************************************
 *  ROS用の各種定義
 */
rcl_publisher_t publisher_odo;
rcl_publisher_t publisher_sensor;
rcl_subscription_t subscriber;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;  // Humble
size_t domain_id = 87;            /////////////////////for_domainID
rcl_timer_t timer;

geometry_msgs__msg__Twist pubmsg_odo;
geometry_msgs__msg__Twist submsg_twist;
std_msgs__msg__Int16MultiArray pubmsg_sensor;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} ros2state;


#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }
#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { init = uxr_millis(); } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)

const int publish_rate = 20;  //publishする周波数[Hz]

void error_loop() {
  while (1) {
    static bool led_brink = true;
    LED(led_brink);
    led_brink = !led_brink;
    Serial.println("ROS ERR");
    delay(1000);
  }
}

void wait_agent() {
  static int64_t last_time = -1;
  if (last_time == -1) { last_time = uxr_millis(); }

  if (uxr_millis() - last_time > 2000) {
    ros2state = (RMW_RET_OK == rmw_uros_ping_agent(200, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
    last_time = uxr_millis();
  }
}

void check_agent() {
  static int64_t last_time = -1;
  if (last_time == -1) { last_time = uxr_millis(); }

  if (uxr_millis() - last_time > 2000 && ros2state == AGENT_CONNECTED) {
    ros2state = (RMW_RET_OK == rmw_uros_ping_agent(200, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
    last_time = uxr_millis();
  }
}

bool ros2_init() {
  allocator = rcl_get_default_allocator();
  Serial.println("FIN rcl_get_default_allocator");
  /////////////////////////////////////////////////////////////////////（旧）
  // //create init_options
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // Serial.println("FIN rclc_support_init");
  /////////////////////////////////////////////////////////////////////（旧）
  ////////////////////////////////////////////////////////////////////////////////////////////////変更箇所（新規）
  // create init_options
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));  // <--- This was missing on ur side
  // Set ROS domain id
  RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));
  // Setup support structure.
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  ////////////////////////////////////////////////////////////////////////////////////////////////変更箇所（新規）

  // create node
  // RCCHECK(rclc_node_init_default(&node, "megarover", "", &support));////////////////////////////変更箇所（旧）
  RCCHECK(rclc_node_init_default(&node, "megarover", "", &support));  //変更箇所（新規）
  Serial.println("FIN rclc_node_init_default");

  // create subscriber
  // RCCHECK(rclc_subscription_init_best_effort(
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "rover_twist"));
  Serial.println("FIN rclc_subscription_init_best_effort");

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_odo,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "rover_odo"));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_sensor,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
    "rover_sensor"));

  // create timer,
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(1000 / publish_rate),
    timer_callback));


  // create executor
  const uint8_t num_handles = 2;  // total number of handles = #subscriptions + #timers
  Serial.println("START rclc_executor_init");
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  Serial.println("FIN rclc_executor_init");
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &submsg_twist, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  // init message memory
  micro_ros_utilities_memory_conf_t conf = { 0 };
  conf.max_ros2_type_sequence_capacity = 2;
  conf.max_basic_type_sequence_capacity = 2;
  if (!micro_ros_utilities_create_message_memory(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
        &pubmsg_sensor,
        conf)) {
    error_loop();
  }

  return true;
}

//destroy entities when re-connect
void destroy_entities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher_odo, &node);
  rcl_publisher_fini(&publisher_sensor, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

//timer_callback
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pubmsg_odo.linear.x = ((v_enc[M_R] + (-1.0 * v_enc[M_L])) / 2.0);
    pubmsg_odo.angular.z = ((v_enc[M_R] - (-1.0 * v_enc[M_L])) / (2.0 * rover_d));
    rcl_ret_t rc = rcl_publish(&publisher_odo, &pubmsg_odo, NULL);
    if (rc == RCL_RET_OK) {
      wrc058.u16Map(MU16_WDT, WDTIME);
    }

    pubmsg_sensor.data.data[0] = wrc058.u16Map(MU16_IM_DI);
    pubmsg_sensor.data.data[1] = (uint16_t)(wrc058.u16Map(MU16_M_VI) * 1000 / 0xfff * 56.1);
    pubmsg_sensor.data.size = pubmsg_sensor.data.capacity;
    RCSOFTCHECK(rcl_publish(&publisher_sensor, &pubmsg_sensor, NULL));
  }
}


//twist message cb
void subscription_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;

  wrc058.s16Map(MS16_S_XS, (int16_t)(msg->linear.x * 1000));
  wrc058.s16Map(MS16_S_ZS, (int16_t)(msg->angular.z * 1000));
  setCtrlMode(MODE_SPD);
  setO_EN(ON_ON);
  wrc058.u16Map(MU16_WDT, WDTIME);
}


/*******************************************
 * セットアップ
 */
void setup() {
  Serial.begin(115200);
  Serial.println("start_setup");

  //モータ制御関連パラメータの設定(適切な値を設定してください)
  std_motor_param[MAX_SPEED] = 1.6;  //最大速度[m/s] (モーターの最大回転速度)
  std_motor_param[MAX_RAD] = 3.14;   //最大旋回速度[rad/s] (無線コントローラーからの入力に対する最大値)
  std_motor_param[MAX_ACC] = 0.7;    //タイヤ回転速度に対する最大加減速度[m/s^2]
  std_motor_param[MAX_V_X] = 1.6;    //X方向最大速度指令値[m/s] (メモリマップの速度指令値最大値制限)
  std_motor_param[MAX_V_Y] = 0.0;    //Y方向最大速度指令値[m/s] (メモリマップの速度指令値最大値制限)
  std_motor_param[MAX_RV_Z] = 3.14;  //Z軸周り最大旋回速度指令値[rad/s] (メモリマップの速度指令値最大値制限)

  //モータ制御パラメータのメモリマップへの反映
  wrc058.memMapClean();
  wrc058.u16Map(MU16_P_SLIMX, std_motor_param[MAX_V_X] * 1000.0);
  wrc058.u16Map(MU16_P_SLIMY, std_motor_param[MAX_V_Y] * 1000.0);
  wrc058.u16Map(MU16_P_RLIM, std_motor_param[MAX_RV_Z] * 1000.0);
  wrc058.u16Map(MU16_P_ALIM, std_motor_param[MAX_ACC] * 1000.0);
  //その他パラメータセットアップ
  setMortorParam2Std();
  setBodyDetail(ROVER_TYPE);

  //ウォッチドッグタイマの設定
  wdTime = WDTIME;

  //タイマー割り込み設定
  setupInterruptTimer();

  //LEDセットアップ
  ledInit();
  LED(1);  //LEDを点ける

//[ROS]接続モード別設定値の設定
#if (WRC_ROS_SERIAL_MODE == MODE_WIFI)
    // using ROS over Wi-Fi
  set_microros_wifi_transports(ssid, password, ROSserverIP, serverPort);
#else
    // using ROS via USB-Serial
  set_microros_transports();
  delay(500);
#endif

  ros2state = WAITING_AGENT;

  LED(0);

  //メモリマップ初期化
  wrc058.initMemmap(19.0);

  delay(50);
  LED(1);

  iMtrL.setDEPin(12);
  iMtrR.setDEPin(12);
  SerialIWS.begin(38400, SERIAL_8N1, 26, 27);

  iMtrL.changeBaud();
  iMtrR.changeBaud();

  SerialIWS.begin(115200, SERIAL_8N1, 26, 27);

  iMtrL.init(ROVER_TYPE);
  iMtrR.init(ROVER_TYPE);

  setDIN(ROVER_TYPE);

#if (ROVER_TYPE == MEGAROVER_3_F40A_C3)
  spiInit();
#endif


  LED(0);

  setO_EN(ON_ON);

#if (ROVER_TYPE == MEGAROVER_3_XBOX)
  NimBLEDevice::init("");
  NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RANDOM);
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */

  /** create new scan */
  NimBLEScan* pScan = NimBLEDevice::getScan();

  /** create a callback that gets called when advertisers are found */
  pScan->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks());

  /** Set scan interval (how often) and window (how long) in milliseconds */
  pScan->setInterval(45);
  pScan->setWindow(15);

  /** Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
  pScan->setActiveScan(true);
  /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
     *  Optional callback for when scanning stops.
     */
  pScan->start(scanTime, scanEndedCB);
  disconnectTime = millis();
#endif


  Serial.println("readey");
  LED(1);  //LEDを点ける
}


/*******************************************
 * メインループ
 */
void loop() {

  int code = NO_INPUT;  //状態判定のためのコード

//XBOXコントローラ接続処理
#if (ROVER_TYPE == MEGAROVER_3_XBOX)
  if (!connected && doConnect) {
    if (advertiseTime < millis()) {
      NimBLEDevice::getScan()->stop();

      /** Found a device we want to connect to, do it now */
      if (connectToServer()) {
        ESP_LOGI(BLETAG, "Success! we should now be getting notifications!");
        File padAddrTxt = SPIFFS.open("/padAddr.txt", "r");

        if (!padAddrTxt) {
          Serial.println("padAddr.txt does not exist.");
          //ESP_LOGI(BLETAG, "padAddr.txt does not exist.");
          padAddrTxt.close();

        } else {
          padAddrTxt.read((uint8_t*)padAddr, padAddrTxt.size());
          padAddrTxt.close();
        }
      } else {
        ESP_LOGI(BLETAG, "Failed to connect, starting scan");
      }
    } else {
      static uint32_t pubmes_time = millis();
      if (millis() - pubmes_time > 1000) {
        printf("Now We get the New Controller Signal and waiting the Paired one. We start connecting after %d msec\n", advertiseTime - millis());
        pubmes_time = millis();
      }
    }

  } else if (!connected && millis() - disconnectTime > 3000 && !NimBLEDevice::getScan()->isScanning()) {
    NimBLEDevice::getScan()->start(scanTime, scanEndedCB);
    doConnect = false;
  } else {
    doConnect = false;
  }
#endif

  //ROS 2接続処理
  switch (ros2state) {
    case WAITING_AGENT:
      wait_agent();
      printf("Wait starting your micro-ros Agent\n");
      break;
    case AGENT_AVAILABLE:
      ros2state = (true == ros2_init()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (ros2state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      if (ros2state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      ros2state = WAITING_AGENT;
      break;
    default:
      break;
  }


  if (Serial.available()) {
    LED(0);
    code = SERIAL_ACCES;
  }

  if (isInterrupt) {
    isInterrupt = false;

    setTimeNow();
    chkPowOffT();
    chkDIN(ROVER_TYPE);

    //モータ電流が制限値を超えた場合、LEDを消灯する
    //isCurrentOver();
    static int currentCnt = 0;
    if (currentFlag) {
      currentCnt++;
      if (currentFlag == currentCnt) {
        currentCnt = 0;
        LED(0);
      } else {
        LED(1);
      }
    }


    //ROS使用時にROSが未接続の場合、速度指令値を0.0[m/s]とする
    if (ros2state != AGENT_CONNECTED) {
      wrc058.s16Map(MS16_S_XS, 0);
      wrc058.s16Map(MS16_S_ZS, 0);
    }


#if (ROVER_TYPE == MEGAROVER_3_F40A_C3)
    if (updatePad() || existsPadInput()) {  //PADの入力処理
      LED(0);
      code = PAD_INPUT;
      chkPadInput();
    } else {
      memCom2V();
    }
#endif
#if (ROVER_TYPE == MEGAROVER_3_XBOX)
    if (updateXboxPad() || existsPadInput()) {  //PADの入力処理
      LED(0);
      code = PAD_INPUT;
      chkPadInput();
    } else {
      memCom2V();
    }

#endif

    ctl2Vcom();

    if (checkWDT() == 0xffff) {
      check_agent();
      LED(0);
    }

    setIMGain();
    updIMGain();
    updO_EN();

    iMtrL.sendMTRSpd(iMtrL.calcTSpd(iMtrL.calcRPMCom(v_com[M_L])));
    iMtrR.sendMTRSpd(iMtrR.calcTSpd(iMtrR.calcRPMCom(v_com[M_R])));

    getEncoderValue();
    getRoverV();
    getMCur();

    getV_IN();
    chkVLevel();

    updO_EN();
    chkLEDFlag();
  }
}
