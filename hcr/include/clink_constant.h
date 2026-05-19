//------------------------------------------------------------------------------
//
// ControLINK-API
// Copyright ⓒ Hanwha Robotics. All rights reserved.
// (https://www.hanwharobotics.co.kr/)
//
// Seokhwan Kim    (seokhwan at hanwha dot com)
// Kazuma   Fukuda (k.fukuda at hanwha dot com)
// Kiho     Cho    (kiho0520 at hanwha dot com)
// Incheol  Jeong  (ic.jeong at hanwha dot com)
// 
//------------------------------------------------------------------------------

#ifndef __CLINK_CONSTANT_H__
#define __CLINK_CONSTANT_H__

#include "clink_def.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef void(*clink_event_callback_t)(
    uint32_t event_grp,
    uint32_t event_subgrp,
    uint32_t event_sender_id,
    int32_t reserve01,
    int32_t reserve02,
    int32_t reserve03,
    int32_t reserve04,
    int32_t reserve05);

/** \addtogroup 40_API 40_API
*  @{
*/
//------------------------------------------------------------------
// 000_CONSTANT
//------------------------------------------------------------------
/** \addtogroup 000_CONSTANT 000_CONSTANT
*  @{
*/


/**
@brief  API RETURN ERROR CODE
*/
typedef enum
{
    /**
    @brief
    @par    설명
            OK
    @par    조치 방법
            Nothing
    */
    CLINK_API_RESULT_OK = 100000,

    //===========================================================================
    // SYSTEM
    //===========================================================================
    /**
    @brief
    @par    설명
            현 버전의 SW 는 해당 API 를 지원하지 않음
    @par    조치 방법
            해당 함수 현재 미지원으로 사용 X
    */
    CLINK_API_RESULT_ERROR_NO_SUPPORT,
    /**
    @brief
    @par    설명
            제어권 획득 실패
    @par    조치 방법
            재시도, 혹은 프로그램 재실행 후 재시도, 반복 시 System 재실행
    */
    CLINK_API_RESULT_ERROR_CONTROL_TAKE_FAILURE,
    /**
    @brief
    @par    설명
            현재 제어권이 없는 상태
    @par    조치 방법
            제어권 획득 후 해당 함수를 호출한다.
            제어SW에 영향을 주는 SET 계열의 API는 제어권이 획득된 상태에서 호출되어야 한다.
    */
    CLINK_API_RESULT_ERROR_YOU_DONT_HAVE_THE_CONTROL,
    /**
    @brief
    @par    설명
            일시적으로 다른 Thread 에서 호출되어 제어권 Control 실패
    @par    조치 방법
            재시도, 혹은 프로그램 재실행 후 재시도, 반복 시 System 재실행
    */
    CLINK_API_RESULT_ERROR_MUTEX_ALREADY_LOCKED,
    /**
    @brief
    @par    설명
            CTRLINK_V3 환경 변수가 존재하지 않음
    @par    조치 방법
            read / write 권한이 있는 디렉토리의 path 를 CTRLINK_V3 라는 이름의 환경변수로 추가한다.
            config.ini 파일에서 사용되는 상대경로의 Root Directory 가 되는 Path 를 환경변수로 등록해야 한다.
    */
    CLINK_API_RESULT_ERROR_CTRLINK_V3_ENV_NOT_EXIST,
    /**
    @brief
    @par    설명
            CTRLINK_V3 환경 변수는 존재하나, 디렉토리가 아니거나 존재하지 않음
    @par    조치 방법
            CTRLINK_V3 환경 변수의 값을 존재하는 디렉토리의 path 로 수정
    */
    CLINK_API_RESULT_ERROR_CTRLINK_V3_ENV_PATH_INCORRECT,
    /**
    @brief
    @par    설명
            Clink API 단의 버전과 제어SW 버전이 일치하지 않음
    @par    조치 방법
            같은 버전으로 새로 설치 및 파일 교체
    */
    CLINK_API_RESULT_ERROR_SW_VER_NOT_MATCHED,
    /**
    @brief
    @par    설명
            System Configuration ini 파일 읽기 실패
    @par    조치 방법
            시스템 초기화 호출 시 ini 파일 경로를 다시 확인한다.
    */
    CLINK_API_RESULT_ERROR_SYSTEM_COULD_NOT_OPEN_INI_FILE,
    /**
    @brief
    @par    설명
            제어SW가 초기화 되지 않은 상태
    @par    조치 방법
            제어SW 초기화 완료 상태에서 프로그램이 시작해야 한다.
            프로그램을 재시작한다. 반복 시 제어SW 역시 다시 실행 후, 프로그램 시작한다.
    */
    CLINK_API_RESULT_ERROR_SYSTEM_CONTROL_SW_NOT_INIT,
    /**
    @brief
    @par    설명
            제어SW가 정상적으로 돌고 있지 않은 상태
    @par    조치 방법
            제어SW가 미실행 또는 비정상적 상황 상태
            제어SW 다시 실행 후, 프로그램 시작한다.
    */
    CLINK_API_RESULT_ERROR_SYSTEM_CONTROL_SW_NOT_RUNNING,
    /**
    @brief
    @par    설명
            OS 내부에서 공유 메모리 생성 실패
    @par    조치 방법
            프로그램을 재시작한다.
    */
    CLINK_API_RESULT_ERROR_SYSTEM_CANNOT_CREATE_SHARED_MEM,
    /**
    @brief
    @par    설명
            제어SW 로 지령된 명령에 대한 응답이 지정된 시간동안 발생하지 않음
    @par    조치 방법
            재시도, 반복 시 Reporting
    */
    CLINK_API_RESULT_ERROR_COMMAND_FUNCTION_NO_RESPONSE,

    //===========================================================================
    // CONTROL BOX
    //===========================================================================
    /**
    @brief
    @par    설명
            Cbox 객체의 ID가 올바르지 않음
    @par    조치 방법
            RPC 환경에서 사용 시 발생할 수 있는 에러로 시스템 연결 시 반환받은 CBox ID를 사용했는 지 체크한다.
            일반적으로 0부터 시작함
    */
    CLINK_API_RESULT_ERROR_CBOX_ID_INVALID = CLINK_API_RESULT_OK + 1000,
    /**
    @brief
    @par    설명
            EtherCAT 통신 Mode 를 Operational Mode 로 변경 불가
    @par    조치 방법
            해당 함수는 Deprecated 되었으며 사용을 금한다.
            사용자 단에서 Op Mode 변경을 하지 않는다.
    */
    CLINK_API_RESULT_ERROR_CBOX_CANNOT_CHANGE_OPERATIONAL_STATE,
    /**
    @brief
    @par    설명
            로봇 생성 시 Cbox 에 해당 Tool IO 는 이미 다른 로봇에 할당되어 있음
    @par    조치 방법
            해당 에러는 미 발생
    */
    CLINK_API_RESULT_ERROR_CBOX_TOOL_IO_ALREADY_ASSIGNED,
    /**
    @brief
    @par    설명
            로봇 생성 시 Cbox 에 해당 Motor 는 이미 다른 로봇에 할당되어 있음
    @par    조치 방법
            로봇 생성 시 모터의 First Index 값을 확인하며, 이전에 다른 로봇 생성은 어떤 모터 Index 를 사용했는지 확인한다.
    */
    CLINK_API_RESULT_ERROR_CBOX_MOTOR_ALREADY_ASSIGNED,
    /**
    @brief
    @par    설명
            입력된 IO 넘버가 올바르지 않음
    @par    조치 방법
            CBox IO 관련 IO 넘버를 확인한다.
    */
    CLINK_API_RESULT_ERROR_CBOX_INVALID_IO_NUMBER,
    /**
    @brief
    @par    설명
            입력된 IO 값이 올바르지 않음
    @par    조치 방법
            CBox IO 관련 IO 값을 확인한다.
            Digital IO의 경우는 1 또는 0 으로만 입력한다.
    */
    CLINK_API_RESULT_ERROR_CBOX_INVALID_IO_VALUE,
    /**
    @brief
    @par    설명
            모든 이중 IO가 할당되어 있음
    @par    조치 방법
            할당 가능한 모든 이중 IO 가 이미 할당되어 있어 더 이상 추가할 수 없는 상태.
            다른 IO 할당 삭제 후 이용
    */
    CLINK_API_RESULT_ERROR_CBOX_REDUNDANT_IO_ALL_ASSIGNED,
    /**
    @brief
    @par    설명
            해당 IO 넘버는 이미 할당되어 있음
    @par    조치 방법
            입력된 IO 넘버로 이미 사용중인 이중 IO가 있으므로 다른 넘버를 이용한다.
    */
    CLINK_API_RESULT_ERROR_CBOX_REDUNDANT_IO_ALREADY_ASSIGNED,
    /**
    @brief
    @par    설명
            삭제하려고 하는 설정된 이중 IO 정보가 없음
    @par    조치 방법
            이중 IO 삭제 시 발생가능한 에러로, 이전에 설정한 IO 넘버를 올바르게 입력했는 지 확인한다.
    */
    CLINK_API_RESULT_ERROR_CBOX_REDUNDANT_IO_NOT_MATCHED,
    /**
    @brief
    @par    설명
            입력된 AMR EMG 넘버가 올바르지 않음
    @par    조치 방법
            AMR EMG 넘버를 확인한다.
    */
    CLINK_API_RESULT_ERROR_CBOX_INVALID_AMR_EMG_NUMBER,

    //===========================================================================
    // ROBOT
    //===========================================================================
    /**
    @brief
    @par    설명
            Robot 객체의 ID가 올바르지 않음
    @par    조치 방법
            로봇 생성 시에 반환받은 Robot ID를 올바르게 입력한다.
            일반적으로 0 부터 시작한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_ID_INVALID = CLINK_API_RESULT_OK + 2000,
    /**
    @brief
    @par    설명
            Robot 생성 실패
    @par    조치 방법
            생성 가능한 로봇의 한도 수를 초과한 경우 발생.
            더 이상 로봇을 추가적으로 생성할 수 없다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_CREATION_FAILURE,
    /**
    @brief
    @par    설명
            Robot Configuration ini 파일 읽기 실패
    @par    조치 방법
            CTRLINK_V3 환경변수 + config 폴더 내 로봇 모델과 동일한 ini 파일(로봇 config 파일)이 존재해야하며 
            해당 파일이 없는 경우 발생한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_COULD_NOT_OPEN_INI_FILE,
    /**
    @brief
    @par    설명
            Robot Configuration ini 파일 내부 양식이 유효하지 않음
    @par    조치 방법
            파일은 존재하지만 내부 설정된 파라미터 양식이 올바르지 못한 경우 발생하며
            내부 파일을 열어서 올바른 양식으로 저장되어 있는 지 확인한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_COULD_NOT_READ_INI_FILE,
    /**
    @brief
    @par    설명
            Robot Configuration ini 파일 내부 파라미터가 유효하지 않음
    @par    조치 방법
            파일 내부 값들이 올바르게 설정되었는지 확인한다.
            특히, min/max 값 설정 관련하여 max가 더 큰 값으로 설정되어야 한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_INVALID_PARAM_IN_INI_FILE,
    /**
    @brief
    @par    설명
            Robot 의 현재 Moving State 는 Reset 이 가능한 상태가 아님
    @par    조치 방법
            해당 Reset 함수는 로봇이 정지된 상태에서 호출되어야 한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_COULD_NOT_RESET_STATE,
    /**
    @brief
    @par    설명
            Servo On 상태에서는 해당 함수를 실행할 수 없음
    @par    조치 방법
            Servo Off 후 해당 기능 실행
    */
    CLINK_API_RESULT_ERROR_ROBOT_SERVO_ON_STATE,
    /**
    @brief
    @par    설명
            설정 가능한 범위 밖의 Robot Parameter 값이 입력 됨
    @par    조치 방법
            각 종 로봇 기능 관련 설정에서 Invalid 한 Parameter 값들이 입력되었을 경우 발생.
            해당 함수에 입력된 Parameter 를 체크한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_PARAM_OUT_OF_RANGE,
    /**
    @brief
    @par    설명
            입력된 세 Point 가 일직선 상에 있음
    @par    조치 방법
            원호, 원 등에 계산되는 3D Point(시작점, 경유점, 종료점)가 일직선 상에 있는 경우 발생.
            포인트 생성을 올바르게 했는 지 점검하고 수정한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_THREE_POINTS_STRAIGHT_LINE,
    /**
    @brief
    @par    설명
            입력된 3점에서 외접원의 중심을 찾을 수 없음
    @par    조치 방법
            원호, 원 등에 계산되는 3D Point(시작점, 경유점, 종료점) 들의 외접원의 중심 해를 찾을 수 없는 경우 발생.
            포인트 생성을 올바르게 했는 지 점검하고 수정한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_OUTER_CIRCLE_NOT_FOUND,
    /**
    @brief
    @par    설명
            로봇 관련 파일 로딩 실패
    @par    조치 방법
            파일 입출력을 필요로 하는 API에서 발생 가능하며 해당 입력된 경로에 파일이 있는 지 검사
    */
    CLINK_API_RESULT_ERROR_ROBOT_FILE_LOAD_FAILURE,
    /**
    @brief
    @par    설명
            로봇 관련 파일 읽기 실패
    @par    조치 방법
            파일 입출력을 필요로 하는 API에서 발생 가능하며 파일은 존재하나 내부 포맷이 Invalid 한 경우 발생.
            파일 내부 저장된 값들을 점검한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_FILE_READ_FAILURE,
    /**
    @brief
    @par    설명
            설정된 Payload 값은 0 미만 일 수 없음
    @par    조치 방법
            로봇의 Payload 는 반드시 0보다 큰 값을 입력한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_GIVEN_PAY_LOAD_IS_LESS_THAN_ZERO,
    /**
    @brief
    @par    설명
            설정된 Payload 값은 0 이므로, TCP 가 없다고 간주되나, Center Of Mass 값은 0 이 아님
    @par    조치 방법
            로봇의 Payload 가 0 인 경우는 Payload 가 없다고 간주되는 형태로 COM 역시 모두 0이어야 한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_GIVEN_PAY_LOAD_IS_ZERO_BUT_CENTER_OF_MASS_NOT_ZERO,
    /**
    @brief
    @par    설명
            설정된 Payload 값이 지원 최대 값 초과
    @par    조치 방법
            로봇 모델에 따라 Max Payload 는 정해져 있으며 더 높게 설정할 수 없다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_GIVEN_PAY_LOAD_MORE_THAN_MAX_CAPACITY,
    /**
    @brief
    @par    설명
            설정된 Center Of Mass 값이 최소 / 최대 임계치 초과
    @par    조치 방법
            일반적으로 +/- 1000mm 값으로 그 사이 값을 입력하도록 하며 COM이 중심으로 너무 먼 Tool 은 사용하지 않도록 한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_GIVEN_PAY_LOAD_CENTER_OF_MASS_OUT_OF_RANGE,
    /**
    @brief
    @par    설명
            입력된 벡터 값이 모두 0
    @par    조치 방법
            벡터 의미를 가지는 API 입력 값이 모두 0으로 입력되었으며 값을 확인한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_VECTOR_SIZE_ZERO,
    /**
    @brief
    @par    설명
            할당 가능한 Workspace 가 없음
    @par    조치 방법
            설정 가능한 Max Workspace 갯수를 초과함
            다른 안전 평면 삭제 후 추가 가능
    */
    CLINK_API_RESULT_ERROR_ROBOT_WORK_SPACE_IS_FULL,
    /**
    @brief
    @par    설명
            해당 ID 의 Workspace 는 등록되어 있지 않음
    @par    조치 방법
            설정되어 있는 올바른 안전 평면 ID를 입력했는 지 확인한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_INVAILD_WORK_SPACE_ID,
    /**
    @brief
    @par    설명
            로봇이 Jog Motion 명령을 실행할 수 없는 상태
    @par    조치 방법
            로봇은 잔여 모션이 없는, 즉 멈춰져 있는 상태에서만 Jog 명령을 내릴 수 있다.
    */
    CLINK_API_RESULT_ROBOT_UNABLE_JOG_STATE,
    /**
    @brief
    @par    설명
            Robot Recording 정보를 저장할 때 다루는 ID 가 Invalid 함
    @par    조치 방법
            버퍼 생성을 통해 반환되는 ID를 올바르게 사용했는 지 값 확인
    */
    CLINK_API_RESULT_ROBOT_INVALID_RECORDING_BUFFER_ID,
    /**
    @brief
    @par    설명
            레코딩 정보 형식이 잘못되었음
    @par    조치 방법
            버퍼 또는 파일 내부 정보가 손상되었음.
            파일인 경우 특히 마지막 부분에 파일 손상된 부분이 없는 지 확인한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_RECORDING_INFO_MODULUS_FAILURE,
    /**
    @brief
    @par    설명
            레코딩 버퍼의 내용이 비어있음
    @par    조치 방법
            비어있는 버퍼이나, 버퍼 내부의 정보를 읽으려고 했을 때 발생
    */
    CLINK_API_RESULT_ERROR_ROBOT_RECORDING_EMPTY_BUFFER,
    /**
    @brief
    @par    설명
            레코딩이 이미 진행중인 상태
    @par    조치 방법
            이미 레코딩이 진행 중인 가운데 다시 시작을 호출했을 때 발생할 수 있음
    */
    CLINK_API_RESULT_ROBOT_ALREADY_RECORDING_RUNNING_STATE,
    /**
    @brief
    @par    설명
            레코딩이 멈춰있는 상태
    @par    조치 방법
            레코딩이 비활성화 된 상태에서 레코딩 종료 기능 호출 시 발생할 수 있음
    */
    CLINK_API_RESULT_ROBOT_ALREADY_RECORDING_STOP_STATE,
    /**
    @brief
    @par    설명
            Interpolation 시 지정된 Speed Factor 범위를 벗어남
    @par    조치 방법
            보간 명령 시, 배속을 결정하는 Factor로 0.1 ~ 2.0 사이 값을 입력해야 하며 이 값을 벗어난 경우 발생
    */
    CLINK_API_RESULT_ERROR_ROBOT_RECORDING_SPEED_FACTOR_OUT_OF_RANGE,
    /**
    @brief
    @par    설명
            Interpolation 생성 실패
    @par    조치 방법
            내부 보간 알고리즘 실행 시 생성 실패.
            입력된 모든 값들을 재 확인
    */
    CLINK_API_RESULT_ERROR_ROBOT_RECORDING_INTERPOLATION_FAILURE,
    /**
    @brief
    @par    설명
            Interpolation 생성 후 Inverse Kinematics 계산 실패
    @par    조치 방법
            입력 Point로 사용되는 모든 위치 및 자세 값들은 로봇 역기구학의 해가 존재해야 한다.
            해가 없는 Point가 포함된 경우 발생
    */
    CLINK_API_RESULT_ERROR_ROBOT_RECORDING_INVERSE_FAILURE,
    /**
    @brief
    @par    설명
            Robot Tool IO 관련 기능 사용 시 입력된 IO 넘버가 올바르지 않음
    @par    조치 방법
            Robot Tool IO 관련 IO 넘버를 확인한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_INVALID_IO_NUMBER,
    /**
    @brief
    @par    설명
            Robot Tool IO 관련 기능 사용 시 입력된 IO 값이 올바르지 않음
    @par    조치 방법
            Robot Tool IO 관련 IO 값을 확인한다.
            Digital IO의 경우 1 또는 0 값을 입력해야 한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_INVALID_IO_VALUE,
    /**
    @brief
    @par    설명
            External Profiler 버퍼가 가득 참
    @par    조치 방법
            버퍼에 내용을 추적하는 속도보다 버퍼에 추가하는 속도가 빠를 경우 버퍼가 가득찰 수 있으며
            지정된 주기를 지키며 버퍼에 추가하는 기능을 이용할 수 있도록 한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_EXT_PROFILER_BUF_FULL,
    /**
    @brief
    @par    설명
            이중 IO 기능 중 Enable Switch On 상태로 현 API 지령 불가
    @par    조치 방법
            Enable Switch 가 활성화된 상태에서는 직접 교시 및 모션 지령등이 불가능한 상태이다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_ENABLE_SWITCH_ON,
    /**
    @brief
    @par    설명
            Tcp Position 추종 설정에 유효하지 않은 Point 갯수 입력
    @par    조치 방법
            3 Point 이상 10 Point 이하의 값을 입력한다.
    */
    CLINK_API_RESULT_ROBOT_TCP_INVALID_NUMBER_OF_ARRAY,
    /**
    @brief
    @par    설명
            입력된 좌표로 많은 수의 가능한 설정이 발생함
    @par    조치 방법
            하나의 추종 가능한 Tcp 정보가 아니라 많은 경우의 Tcp 정보의 해를 찾은 경우 발생.
            입력한 Point List 들을 하나의 해만 나오도록 수정해야 한다.
    */
    CLINK_API_RESULT_ROBOT_TCP_HAS_SEVERAL_SOLUTIONS,
    /**
    @brief
    @par    설명
            Robot 이 현재 움직이는 상태로 지령 불가
    @par    조치 방법
            로봇의 설정을 변경하는 해당 기능은 로봇이 정지해 있는 상태에서만 호출되어야 한다.
    */
    CLINK_API_RESULT_ROBOT_IS_MOVING_STATE,
    /**
    @brief
    @par    설명
            Robot 의 통신연결이 되어있지 않은 상태
    @par    조치 방법
            Robot 의 Servo On/Off 하는 경우는 특별히 로봇의 EtherCAT 통신 연결 상태 또한 체크하며
            해당 에러 발생 시 통신에 문제가 있는 상태.
            일반적으로 다양한 에러 이벤트를 통해 에러가 발생한 상황
    */
    CLINK_API_RESULT_ROBOT_ECAT_NOT_CONNECTED,
    /**
    @brief
    @par    설명
            로봇 영점 기능 설정 후 로봇으로부터 응답이 없음
    @par    조치 방법
            재시도, 프로그램 재실행 후 재시도, 재부팅 후 재시도.
            반복 시, 관리자에게 문의한다.
    */
    CLINK_API_RESULT_ROBOT_HOME_RESET_FAILURE_NO_RESPONSE,
    /**
    @brief
    @par    설명
            로봇 영점 기능 설정 관련 파라미터 설정 실패
    @par    조치 방법
            재시도, 프로그램 재실행 후 재시도, 재부팅 후 재시도.
            반복 시, 관리자에게 문의한다.
    */
    CLINK_API_RESULT_ROBOT_HOME_RESET_FAILURE_INVALID_PARAMETER,
    /**
    @brief
    @par    설명
            Cbox 와 Robot 의 Simulation/REal Type 이 불일치
    @par    조치 방법
            둘 다 SIM Type 또는 둘다 REAL Type 이어야 함
            시스템 및 로봇 생성 인자로 둘 모두 Type 을 일치시켜 준다.
    */
    CLINK_API_RESULT_ROBOT_TYPE_NOT_MATCHED_WITH_CBOX,
    /**
    @brief
    @par    설명
            Friction Estimation File Save 실패(로봇 Body 내 저장 시 Cbox 의 파일 읽기 실패)
    @par    조치 방법
            재시도, 프로그램 재실행 후 재시도, 재부팅 후 재시도.
            반복 시, 관리자에게 문의한다.
    */
    CLINK_API_RESULT_ROBOT_DYNAMICS_CALIBRATION_FILE_SAVE_FAILURE,
    /**
    @brief
    @par    설명
            Friction Estimation 실패
    @par    조치 방법
            Friction Estimation 도중 다양한 원인에 의해서 로봇이 정지하였을 경우 발생
    */
    CLINK_API_RESULT_ROBOT_DYNAMICS_CALIBRATION_FAILURE,
    /**
    @brief
    @par    설명
            Dry Run 시작 실패
    @par    조치 방법
            내부 Dry Run 실행을 위한 자원 획득 실패로, 다른 Dry Run 을 다수 호출한 경우 발생 가능함
    */
    CLINK_API_RESULT_ROBOT_DYNAMICS_DRY_RUN_START_FAILURE,
    /**
    @brief
    @par    설명
            Path Recording Playback 시 출발 위치가 다름
    @par    조치 방법
            현재 로봇의 위치는 Playback 의 출발 위치에 반드시 있어야 한다.
            로봇의 위착 다를 경우, 레코딩된 데이터의 시작 각도를 읽어 출발 위치로 보낸 후 Playback 한다.
    */
    CLINK_API_RESULT_ROBOT_TARGET_START_POSE_NOT_MATCHED,
    /**
    @brief
    @par    설명
            Robot 생성 시, API 단에서 Load 하는 Kinematics Module 을 찾을 수 없음
    @par    조치 방법
            로드한 config.ini 파일 내 Kinematics Path 확인.
            해당 Path 에 실 Kinematics Module 이 존재하는 지 확인
    */
    CLINK_API_RESULT_ROBOT_CLIENT_KINEMATICS_MODULE_LOAD_FAILURE,
    /**
    @brief
    @par    설명
            Robot 생성 시, 제어SW 단에서 Load 하는 Kinematics Module 을 찾을 수 없음
    @par    조치 방법
            로드한 config.ini 파일 내 Kinematics Path 확인.
            해당 Path 에 실 Kinematics Module 이 존재하는 지 확인
    */
    CLINK_API_RESULT_ROBOT_SERVER_KINEMATICS_MODULE_LOAD_FAILURE,
    /**
    @brief
    @par    설명
            설정된 Payload와 측정된 Payload 값 차이가 큼
    @par    조치 방법
            실제 Payload 및 설정되어 있는 Payload를 다시 확인한다.
    */
    CLINK_API_RESULT_ROBOT_STATIC_PAYLOAD_VALICATION_FAILURE,

    //===========================================================================
    // ROBOT JOINT
    //===========================================================================
    /**
    @brief
    @par    설명
            입력된 Joint 의 개수와 로봇의 Joint 개수가 일치하지 않음
    @par    조치 방법
            일반적으로 협동 로봇의 Joint 수는 6이다.
            입력 값을 확인한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_NUMBER_OF_JOINT_NOT_MATCHED = CLINK_API_RESULT_OK + 3000,
    /**
    @brief
    @par    설명
            입력된 Joint Index 가 로봇의 범위 밖임
    @par    조치 방법
            Joint Index 는 0 부터 시작하며, Joint Index 범위를 초과한 값을 입력했는 지 확인한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_JOINT_INDEX_OUT_OF_RANGE,
    /**
    @brief
    @par    설명
            로봇 Joint 설정 관련하여 Invalid 한 Parameter를 입력 했을 때 발생
    @par    조치 방법
            설정 가능한 범위 내의 값을 올바르게 입력했는 지 확인한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_JOINT_PARAM_OUT_OF_RANGE,

    //===========================================================================
    // ROBOT MOTION
    //===========================================================================
    /**
    @brief
    @par    설명
            Motion Command 생성 시 Unique 한 Command ID 를 생성할 수 없는 경우
    @par    조치 방법
            과도한 생성으로 인해 발생할 수 있으며, 모션 생성과 모션 종료 함수는 항상 짝이되도록 사용해야 한다.
    */
    CLINK_API_RESULT_ERROR_COULD_NOT_CREATE_UNIQUE_COMMAND_ID = CLINK_API_RESULT_OK + 4000,
    /**
    @brief
    @par    설명
            Motion Command 정보를 가져오기 위해선 Robot ID 가 처음으로 설정되어야 함
    @par    조치 방법
            모션 생성 API 호출 직후, 해당 모션의 로봇 ID를 설정하는 API 호출을 하고나서
            다른 모션 관련 설정 API를 호출할 수 있도록 한다.
    */
    CLINK_API_RESULT_ERROR_ROBOT_ID_MUST_BE_SET_FIRST,
    /**
    @brief
    @par    설명
            입력된 Motion Command ID 정보를 찾을 수 없음
    @par    조치 방법
            입력한 Motion Command ID를 점검한다.
            모션 생성 API 를 통해 반환받은 ID 를 사용해야 하며, 모션 종료 API 호출 이후
            모션 ID는 Invalid 하다.
    */
    CLINK_API_RESULT_ERROR_COULD_NOT_FIND_COMMAND_ID,
    /**
    @brief
    @par    설명
            입력된 Motion Profiler 정보를 찾을 수 없음
    @par    조치 방법
            현재 지원 가능한 Type 은 CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE 이며
            해당 파라미터만을 사용하도록 한다.
    */
    CLINK_API_RESULT_ERROR_COULD_NOT_FIND_PROFILER_TYPE,
    /**
    @brief
    @par    설명
            설정 가능한 범위 밖의 Motion Parameter 값이 입력 됨
    @par    조치 방법
            입력한 Motion Parameter 값을 체크한다.
    */
    CLINK_API_RESULT_ERROR_MOTION_COMMAND_OUT_OF_RANGE,
    /**
    @brief
    @par    설명
            해당 Motion Command Type 에서는 블렌딩 설정이 불가함
    @par    조치 방법
            일반적인 TCP 및 Joint 모션 Type 에서만 블렌딩 기능을 사용한다.
    */
    CLINK_API_RESULT_ERROR_COULD_NOT_SET_BLENDING_MODE,
    /**
    @brief
    @par    설명
            컨베이어 진행 중 삽입할 수 없는 모션이 들어옴
    @par    조치 방법
            컨베이어 모드 상태에서는 일반적인 TCP 모션만을 추가할 수 있다.
    */
    CLINK_API_RESULT_ERROR_MOTION_CANNOT_INSERT_IN_CONVEYOR,
    /**
    @brief
    @par    설명
            모션 IO 설정에서 해당 Type 는 설정할 수 없음
    @par    조치 방법
            시작 조건 설정에서 Digital IO 만 설정할 수 있다.
    */
    CLINK_API_RESULT_ERROR_MOTION_CANNOT_SET_IO_TYPE,
    /**
    @brief
    @par    설명
            모션을 추가할 수 없는 상태
    @par    조치 방법
            모션은 Servo On, Position Mode(직접 교시 모드 X), 비상 정지 상황이 아닌 경우, 
            Safeguard 정지가 아닌 경우, 충돌 상태가 아닌 경우에만 추가 할 수 있다.
    */
    CLINK_API_RESULT_ERROR_MOTION_UNINSERTABLE_STATE,
    /**
    @brief
    @par    설명
            모션 프로파일링 실패
    @par    조치 방법
            모션 파라미터를 확인한다. 특히 시작/끝 속도가 존재하고 거리가 짧은 경우 발생할 수 있다.
            해당 모션 파라미터로 프로파일링을 할 수 없을 때 발생한다.
            해당 상황의 경우 모션 파라미터를 수정한다.
    */
    CLINK_API_RESULT_ERROR_MOTION_PROFILING_FAILURE,
    /**
    @brief
    @par    설명
            모션 도중 각도 위반 발생
    @par    조치 방법
            Joint 가 아닌 모션 Type 에서 모션 진행 시 안전 각도 범위가 벗어난 경우 미리 검사하여 에러를 반환한다.
            모션 파라미터(Target 위치 및 자세)를 수정해야 한다.
    */
    CLINK_API_RESULT_ERROR_MOTION_OUT_OF_ANGLE_RANGE,
    /**
    @brief
    @par    설명
            모션 도중 Singular Point 존재
    @par    조치 방법
            Joint 가 아닌 모션 Type 에서 모션 진행 중 역기구학의 해가 없는 Target 으로 이동 시도하는 Point 포함 시 발생.
            모션 파라미터(Target 위치 및 자세)를 수정해야 한다.
    */
    CLINK_API_RESULT_ERROR_MOTION_SINGULARITY_POINT,
    /**
    @brief
    @par    설명
            자동 감속 지정 실패
    @par    조치 방법
            Joint 가 아닌 모션 Type 에서 속도/가속도 위반 하는 경우 자동으로 조금씩 속도를 줄여가면서
            위반 상황이 발생하지 않는 속도를 찾으나, 매우 낮은 속도를 낮춘 경우에도 위반이 발생하는 경우
            해당 에러 발생할 수 있으며, 파라미터 수정이 필요하다.
    */
    CLINK_API_RESULT_ERROR_MOTION_AUTO_SPEED_DOWN_FAILURE,
    /**
    @brief
    @par    설명
            Arc, Circular, Circle Motion 에서 두 위치가 같음
    @par    조치 방법
            시작점, 경유점, 도착점의 Point를 점검하여 겹치는 Point 가 없도록 수정한다.
    */
    CLINK_API_RESULT_ERROR_MOTION_TWO_POSITION_SAME,
    /**
    @brief
    @par    설명
            시작 속도가 있는 모션은 이전 모션의 진행방향과 현재 모션의 진행방향이 일치해야 한다.
    @par    조치 방법
            이전 모션의 끝 속도와 현재 모션의 시작 속도가 존재하는 모션은 멈추지 않고 이동하는 형태로
            부드럽게 모션이 이어져야 하며, 이전 모션의 마지막 진행 방향과 다음 모션의 첫 진행 방향을 비교하여
            차이가 날 때 에러를 반환한다.
    */
    CLINK_API_RESULT_ERROR_MOTION_DIRECTION_VECTOR_NOT_SAME,
    /**
    @brief
    @par    설명
            사용자가 지정한 모션의 Target 위치가 설정된 workspace 범위를 벗어남
    @par    조치 방법
            가능한 경우 workspace 설정 변경 또는 위반 되지 않는 위치로 모션 Target 위치 변경
    */
    CLINK_API_RESULT_ERROR_MOTION_TARGET_POS_WORKSPACE_VIOLATION,

    //===========================================================================
    // USER MEMORY
    //===========================================================================
    /**
    @brief
    @par    설명
            저장소에서 키를 찾을 수 없거나, 접근에 문제가 생긴 경우 발생
    @par    조치 방법
            키 값 입력 확인
    */
    CLINK_API_RESULT_ERROR_KEY_NOT_FOUND = CLINK_API_RESULT_OK + 5000,
    /**
    @brief
    @par    설명
            저장소가 가득찬 경우 발생
    @par    조치 방법
            불필요한 저장소 삭제 후 재 생성
    */
    CLINK_API_RESULT_ERROR_KEY_MAX_MEMORY,

    /*===========================================================================*/
    // 통신 에러
    // Modbus, EIP, Profinet, EtherCAT, IO LINK
    /*===========================================================================*/
    /**
    @brief
    @par    설명
            모드버스 Client 통신 간 예외 상황 발생
    @par    조치 방법
            Client IP, Port 확인
    */
    CLINK_API_RESULT_MODBUS_EXCEPTION_ERROR = CLINK_API_RESULT_OK + 6000,
    /**
    @brief
    @par    설명
            모드버스 Request 실패
    @par    조치 방법
            Server IP, Port 확인, Client Name 확인
    */
    CLINK_API_RESULT_MODBUS_REQUEST_FAIL,
    /**
    @brief
    @par    설명
            모드버스 Input Error
    @par    조치 방법
            모드버스 Input parameter 확인
    */
    CLINK_API_RESULT_MODBUS_INPUT_ERROR,
    /**
    @brief
    @par    설명
            모드버스 Client Name Error
    @par    조치 방법
            모드버스 Client Name 중복 또는 검색 실패
    */
    CLINK_API_RESULT_MODBUS_CLIENT_NAME_ERROR,
    /**
    @brief
    @par    설명
            모드버스 Client 추가 실패
    @par    조치 방법
            입력 값 확인
    */
    CLINK_API_RESULT_MODBUS_CLIENT_ADD_FAIL,
    /**
    @brief
    @par    설명
            모드버스 Client IO Name Error
    @par    조치 방법
            모드버스 Client IO Name 중복 또는 검색 실패
    */
    CLINK_API_RESULT_MODBUS_CLIENT_IO_NAME_ERROR,
    /**
    @brief
    @par    설명
            모드버스 Client IO 추가 실패
    @par    조치 방법
            입력 값 확인
    */
    CLINK_API_RESULT_MODBUS_CLIENT_IO_ADD_FAIL,
    /**
    @brief
    @par    설명
            모드버스 Client Illegal Function Type
    @par    조치 방법
            Value Number 범위 실패 시 발생
    */
    CLINK_API_RESULT_MODBUS_CLIENT_EXCEPTION_ILLEGAL_FUNC_TYPE,
    /**
    @brief
    @par    설명
            모드버스 Client Illegal Data Address
    @par    조치 방법
            주소 지정 실패 시 발생
    */
    CLINK_API_RESULT_MODBUS_CLIENT_EXCEPTION_ILLEGAL_DATA_ADDRESS,
    /**
    @brief
    @par    설명
            모드버스 Client Illegal Command
    @par    조치 방법
            Function Code 오류
    */
    CLINK_API_RESULT_MODBUS_CLIENT_EXCEPTION_ILLEGAL_CMD,
    /**
    @brief
    @par    설명
            Ethernet/IP Request 실패
    @par    조치 방법
            Server IP, Port 확인, Client Name 확인
    */
    CLINK_API_RESULT_EIP_REQUEST_FAIL,
    /**
    @brief
    @par    설명
            Profinet Request 실패
    @par    조치 방법
            현재 미발생
    */
    CLINK_API_RESULT_PROFINET_REQUEST_FAIL,
    /**
    @brief
    @par    설명
            EtherCAT SDO read 실패
    @par    조치 방법
            재시도, 혹은 프로그램 재실행 후 재시도, 반복 시 System 재실행
    */
    CLINK_API_RESULT_ERROR_SDO_READ_FAILURE,
    /**
    @brief
    @par    설명
            EtherCAT SDO read 무응답
    @par    조치 방법
            재시도, 혹은 프로그램 재실행 후 재시도, 반복 시 System 재실행
    */
    CLINK_API_RESULT_ERROR_SDO_READ_NO_RESPONSE,
    /**
    @brief
    @par    설명
            입력된 IO LINK의 WP 넘버가 올바르지 않음.
    @par    조치 방법
            IO LINK의 WP Number 확인
    */
    CLINK_API_RESULT_ERROR_IO_LINK_INVALID_WP_NUMBER,
    /**
    @brief
    @par    설명
            입력된 IO LINK의 Device 넘버가 올바르지 않음.
    @par    조치 방법
            IO LINK의 Device Number 확인
    */
    CLINK_API_RESULT_ERROR_INVALID_DEVICE_ID_NUMBER,
    /**
    @brief
    @par    설명
            입력된 IO LINK의 Device Data Size가 올바르지 않음
    @par    조치 방법
            IO LINK의 Device Data Size 확인
    */
    CLINK_API_RESULT_ERROR_INVALID_DATA_SIZE,

    //===========================================================================
    // UNKNOWN
    //===========================================================================
    /**
    @brief
    @par    설명
            Exception 이 발생 상황 에러
    @par    조치 방법
            현재 미발생
    */
    CLINK_API_RESULT_ERROR_UNKNOWN = 190000,
    /**
    @brief
    @par    설명
            SW 적 비정상적 동작 시 발생
    @par    조치 방법
            재시도, 반복 발생 시 리포트
    */
    CLINK_API_RESULT_ERROR_CTRLINK_INTERNAL_ERROR,
    /**
    @brief
    @par    설명
            CLINK_API_RESULT_ERROR_CTRLINK_INTERNAL_ERROR 발생 후 리셋하지 않은 상태로 함수 호출
    @par    조치 방법
            재시도, 반복 발생 시 리포트
    */
    CLINK_API_RESULT_ERROR_PREV_CTRLINK_INTERNAL_ERROR_NOT_YET_RESOLVED,
    /**
    @brief
    @par    설명
            함수 호출 후, 1초 이상 FW 로 부터 함수 처리에 대한 Ack 수신을 못함
    @par    조치 방법
            재시도, 반복 발생 시 리포트
    */
    CLINK_API_RESULT_ERROR_CALL_TIMEOUT,
    /**
    @brief
    @par    설명
            RPC 환경에서 Config 파일 로딩 실패
    @par    조치 방법
            재시도, 반복 발생 시 리포트
    */
    CLINK_API_RESULT_ERROR_CONFIG_FILE_LOAD_FAILURE,

    /*===========================================================================*/
    // 이하는 warning
    // 즉, 잘못된 호출이지만 프로그램을 실행하는데 문제없음.
    /*===========================================================================*/
    /**
    @brief
    @par    설명
            Error 가 아닌 Warning 코드
    @par    조치 방법
            발생하더라도 흐름에 영향을 주지 않는다고 판단되는 코드(미발생 코드)
    */
    CLINK_API_RESULT_WARNING_BEGIN = 200000,
    /**
    @brief
    @par    설명
            시스템이 이미 생성되어 있는상태
    @par    조치 방법
            이미 시스템 초기화 되어 있는 상태에서 시스템 초기화 호출 시 발생
    */
    CLINK_API_RESULT_WARNING_SYSTEM_ALREADY_CREATED,
    /**
    @brief
    @par    설명
            이미 제어권을 가지고 있는 상태
    @par    조치 방법
            이미 제어권을 가지고 있는 상태에서 제어권 획득 API 호출 시 발생
    */
    CLINK_API_RESULT_WARNING_YOU_DONT_HAVE_THE_CONTROL,
    /**
    @brief  NULL 이 입력된 상태. default 파라메터를 사용.
    */
    /**
    @brief
    @par    설명
            API 호출에 NULL 이 입력된 상태
    @par    조치 방법
            내부 Default 값 사용
    */
    CLINK_API_RESULT_WARNING_NULLPTR_IS_GIVEN_DEFAULT_VALUE_IS_USED,
    /**
    @brief
    @par    설명
            이미 제어권을 반납한 상태
    @par    조치 방법
            이미 제어권을 반납한 상태에서 제어권 Release API 호출 시 발생
    */
    CLINK_API_RESULT_WARNING_YOU_ALREADY_RELEASED_THE_CONTROL,
    /**
    @brief
    @par    설명
            Motion Queue 에 채워진 모션 파라미터가 없는 상태
    @par    조치 방법
            Motion Queue 가 비어있는 상태에서 flush 혹은 clear 시도
    */
    CLINK_API_RESULT_WARNING_MOTION_COMMAND_PARAMETERS_ARE_EMPTY,
    /**
    @brief
    @par    설명
            로봇의 Inverse Kinematics 계산 실패
    @par    조치 방법
            단순 계산 API 호출에서 역기구학 해 생성 실패 시 발생
    */
    CLINK_API_RESULT_WARNING_ROBOT_INVERSE_FAILURE,
    /**
    @brief
    @par    설명
            Workspace 가 이미 삭제된 상태
    @par    조치 방법
            이미 삭제된 상태에서 다시 삭제 시도시 발생
    */
    CLINK_API_RESULT_WARNING_ROBOT_WORK_SPACE_ALREADY_DELETED,
    /**
    @brief
    @par    설명
            컨베이어 엔코더 미사용 상태에서 관련 API 호출
    @par    조치 방법
            해당 에러 시 호출 API 동작(계산) 안함
    */
    CLINK_API_RESULT_WARNING_ROBOT_CONVEYOR_ENCODER_NOT_SETTING,
    /**
    @brief
    @par    설명
            같은 상태의 EtherCAT Operational Mode
    @par    조치 방법
            호출 함수 Deprecated
    */
    CLINK_API_RESULT_WARNING_CBOX_OPERATION_MODE_ALREADY_CHANGED,
    /**
    @brief
    @par    설명
            다른 프로세스에서 만든 Cbox 와 모델명이 다름
    @par    조치 방법
            같은 Cbox 모델로 새로 초기화 한다.
    */
    CLINK_API_RESULT_WARNING_CBOX_NAME_DIFFERENT,
    /**
    @brief
    @par    설명
            입력된 Motion Command ID 정보를 찾을 수 없음
    @par    조치 방법
            Motion Command 해제 API 호출 시 생성 안 된 ID 사용할 때 발생
    */
    CLINK_API_RESULT_WARNING_COULD_NOT_FIND_COMMAND_ID
}CLINK_API_RESULT;

/**
@brief  Event 의 대표 그룹 모음이다. \n
        이 enumeration 의 각 event group 은 \n
        CLINK_EVENT_SUBGRP 으로 시작하는 subgroup \n
        들을 통해서 더 다양한 정보를 표기한다.
*/
typedef enum
{
    /**
    @brief  에러 감지 시, 발생한다. \n
            해당 이벤트 발생 시 로봇은 긴급정지, servo off 를 실행한다. \n
            CLINK_EVENT_SUBGRP_ERROR 참조
    */
    CLINK_EVENT_GRP_ERROR = 301000,
    /**
    @brief  모션 커맨드의 완료 등이 되었을 때 발생한다. \n
            CLINK_EVENT_SUBGRP_MOTION_COMMAND 참조
    */
    CLINK_EVENT_GRP_MOTION_COMMAND = 302000,
    /**
    @brief  Safety 설정을 위반한 경우 발생한다. \n
            해당 이벤트 발생 시 로봇은 긴급정지, servo off 를 실행한다. \n
            CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED 참조
    */
    CLINK_EVENT_GRP_ROBOT_SAFETY_VIOLATED = 303000,
    /**
    @brief  기구적 설계 SPEC 을 벗어난 경우 발생한다. \n
            현재 해당 이벤트는 발생하지 않는다.
            CLINK_EVENT_SUBGRP_ROBOT_SPEC_VIOLATED 참조
    */
    CLINK_EVENT_GRP_ROBOT_SPEC_VIOLATED = 304000,
    /**
    @brief  충돌감지를 한 경우 발생한다. \n
            CLINK_EVENT_SUBGRP_ROBOT_COLLISION_DETECTED 참조
    */
    CLINK_EVENT_GRP_ROBOT_COLLISION_DETECTED = 305000,
    /**
    @brief  이중 IO 관련 event
    */
    CLINK_EVENT_GRP_ROBOT_REDUNDANT_IO = 306000,
    /**
    @brief  컨트롤 박스에 연결된 버튼의 상태가 바뀐 경우 발생한다.  \n
            CLINK_EVENT_SUBGRP_CONTROL_BOX_BUTTON_CHANGED 참조.
    */
    CLINK_EVENT_GRP_CONTROL_BOX_BUTTON_CHANGED = 307000,
    /**
    @brief  각종 통지 기능을 담당한다.
    */
    CLINK_EVENT_GRP_NOTIFICATION = 308000,
    /**
    @brief  modbus 관련된 이벤트
    */
    CLINK_EVENT_GRP_MODBUS_NOTI = 309000
}CLINK_EVENT_GRP;

/**
@brief  CLINK_EVENT_GRP_ERROR 와 함께 발생하는 subgroup
*/
typedef enum
{
    CLINK_EVENT_SUBGRP_ERROR_INVALID = CLINK_EVENT_GRP_ERROR + 1,
    /**
    @brief
    @par    설명
            컨트롤 박스와 로봇의 연결이 단절된 경우 발생 \n
            EtherCAT Master 에서 감시하는 에러 체크 로직
    @par    Sender ID
            컨트롤 박스 ID
    @par    조치 방법
            EtherCAT Master 에서 감지된 에러로, EthcerCAT 네트워크 구성 및 연결 상태를 점검한다. \n
            일반적으로 다른 EtherCAT 관련 에러와 함께 발생한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_CONTROL_BOX_ROBOT_DISCONNECTED,
    /**
    @brief
    @par    설명
            [모터 공통] 모터 Slave 에러 코드 및 Status Word Fault Bit 트리거 시 발생
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            추가 파라미터로 전달되는 코드 값 참조하여 모터 EtherCAT Slave 점검
    */
    CLINK_EVENT_SUBGRP_ERROR_ECAT_SLAVE_FAULT,
    /**
    @brief
    @par    설명
            [EtherCAT Slave 공통] EtherCAT Slave PDO 통신의 에러가 발생한 경우
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            EtherCAT Slave HW 연결 상태 점검
    */
    CLINK_EVENT_SUBGRP_ERROR_ECAT_SLAVE_PDO_FAULT,
    /**
    @brief
    @par    설명
            [EtherCAT Slave 공통] EtherCAT Slave SDO 통신의 에러가 발생한 경우
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            EtherCAT Slave HW 연결 상태 점검
    */
    CLINK_EVENT_SUBGRP_ERROR_ECAT_SLAVE_SDO_FAULT,
    /**
    @brief
    @par    설명
            [모터 공통] 모터 Slave 파워 비 인가 시 발생
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            모터 EtherCAT Slave HW 연결 상태 점검 및 분석
    */
    CLINK_EVENT_SUBGRP_ERROR_ECAT_SLAVE_VOLTAGE_DISABLED,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 모터 EtherCAT Slave 사에서 제공하는 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            추가 파라미터로 전달되는 코드 값 참조하여 모터 EtherCAT Slave 점검
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_DRIVE_INTERNAL_ERROR,
    /**
    @brief
    @par    설명
            [AMC] 모터 Short Circuit 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_SHORT_CIRCUIT,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 모터 Under Voltage 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC 및 Welcon Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_UNDER_VOLTAGE,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 모터 Over Voltage 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC 및 Welcon Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_OVER_VOLTAGE,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 모터 Over Temperature 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC 및 Welcon Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_DRIVE_OVER_TEMPERATURE,
    /**
    @brief
    @par    설명
            [AMC] 모터 Parameter Restore 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_PARAMETER_RESTORE_ERROR,
    /**
    @brief
    @par    설명
            [AMC] 모터 Parameter Store 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_PARAMETER_STORE_ERROR,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 모터 Hall State 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC 및 Welcon Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_INVALID_HALL_STATE,
    /**
    @brief
    @par    설명
            [AMC] 모터 Phase Sync 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_PHASE_SYNC_ERROR,
    /**
    @brief
    @par    설명
            [AMC] 모터 Phase Detection Fault 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_PHASE_DETECTION_FAULT,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 모터 Feedback Sensor Error 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC 및 Welcon Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_FEEDBACK_SENSOR_ERROR,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 모터 Over Speed 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC 및 Welcon Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_MOTOR_OVER_SPEED,
    /**
    @brief
    @par    설명
            [AMC] 모터 Under Voltage 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_USER_UNDER_VOLTAGE,
    /**
    @brief
    @par    설명
            [AMC] 모터 Over Voltage 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_USER_OVER_VOLTAGE,
    /**
    @brief
    @par    설명
            [AMC] 모터 Non Sinusoidal Communication 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_NON_SINUSOIDAL_COMMUTATION,
    /**
    @brief
    @par    설명
            [AMC] 모터 Velocity Following 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_VELOCITY_FOLLOWING_ERROR,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 모터 Position Following 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC 및 Welcon Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_POSITION_FOLLOWING_ERROR,
    /**
    @brief
    @par    설명
            [WELCON] 모터 Communication 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Welcon Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_COMMUNICATION_ERROR,
    /**
    @brief
    @par    설명
            [WELCON] 모터 Over Current 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Welcon Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_OVER_CURRENT_ERROR,
    /**
    @brief
    @par    설명
            [WELCON] 모터 Encoder 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Welcon Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ENCODER_ERROR,
    /**
    @brief
    @par    설명
            [LSM] 모터 EtherCAT Slave 사에서 제공하는 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            추가 파라미터로 전달되는 코드 값 참조하여 모터 EtherCAT Slave 점검 \n
            현재 해당 에러 이벤트는 미발생 상태
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_HW_FAULT,
    /**
    @brief
    @par    설명
            EtherCAT IO Slave 에서 Short 발생
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            추가 파라미터로 전달되는 코드 값 참조하여 EtherCAT IO Slave 점검 \n
    */
    CLINK_EVENT_SUBGRP_ERROR_IO_BOARD_SHORT_DETECTION,
    /**
    @brief
    @par    설명
            Safety Module 에서 발생하는 STOP0 상태 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Safety Module LED 코드 및 Safety Module 메뉴얼 참조하여 처리한다. \n
            (Deprecated, Safety Module 에러 이벤트는 세부적으로 변경)
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_SLAVE_STOP0_REQUEST,
    /**
    @brief
    @par    설명
            Safety Module 에서 발생하는 STOP1 상태 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Safety Module LED 코드 및 Safety Module 메뉴얼 참조하여 처리한다. \n
            Deprecated, Safety Module 에러 이벤트는 세부적으로 변경)
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_SLAVE_STOP1_REQUEST,
    /**
    @brief
    @par    설명
            Safety Module 에서 발생하는 STOP2 상태 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Safety Module LED 코드 및 Safety Module 메뉴얼 참조하여 처리한다. \n
            Deprecated, Safety Module 에러 이벤트는 세부적으로 변경)
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_SLAVE_STOP2_REQUEST,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            릴레이 불량 판정
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Control Box 릴레이 점검
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_RELAY_FAILURE,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            긴급정지 라인 불량 판정
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            긴급정지 버튼 케이블 및 핀 점검
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_EMERGENCY_SWITCH_FAILURE,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            Energy Eater Device 불량 판정
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Energy Eater Device 점검
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_ENERGY_EATER_DEVICE_FAILURE,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            MC Relay Test 실패
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Safety Module LED 코드 및 Safety Module 메뉴얼 참조하여 처리한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_MC_RELAY_NORMAL_CLOSE_TEST_FAILURE,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            MCU Hardware Failure
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Safety Module LED 코드 및 Safety Module 메뉴얼 참조하여 처리한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_MCU_HW_FAILURE,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            48V DC Supply 관련 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Safety Module LED 코드 및 Safety Module 메뉴얼 참조하여 처리한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_48V_DC_SUPPLY_FAILURE,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            Position Following Error
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            제어SW 측에서 더욱 좁은 범위로 감시하기 때문에 제어SW에서 먼저 Detection 되어야 정상 \n
            HW Actual Position 값이 정상적으로 들어오는 지 점검한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_JOINT_POSITION_FOLLOWING_ERROR,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            Velocity Following Error
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            HW Actual Velocity 값이 정상적으로 들어오는 지 점검한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_JOINT_VELOCITY_FOLLOWING_ERROR,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            Motor 각도 범위를 벗어남
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Motor Tool 을 이용하여 범위내로 이동
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_JOINT_POSITION_RANGE_ERROR,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            Motor 각속도 범위를 벗어남
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            반복 발생 시, Motor Tuning 및 데이터 진단 필요
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_JOINT_VELOCITY_RANGE_ERROR,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            Motor 토크 범위를 벗어남
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            반복 발생 시, 모터 점검
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_JOINT_TORQUE_RANGE_ERROR,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            TCP 끝 단의 Command 와 Actual 값이 차이가 발생
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            반복 발생 시, Actual Position 및 Position Following Error 점검
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_TCP_POSITION_FOLLOWING_ERROR,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            EtherCAT Connection 실패
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            HW 배선 점검. Safety Module LED 코드 및 Safety Module 메뉴얼 참조하여 처리한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_ETHERCAT_CONNECTION_FAILURE,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 경고 \n
            Motor 각가속도 범위를 벗어남
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            동작에 이상 없음
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_ETHERCAT_CRC_MISMATCH_ERROR,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            TCP 끝 단의 Command 와 Actual 값이 차이가 발생
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            반복 발생 시, Motor Tuning 및 데이터 진단 필요
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_JOINT_ACCELERATION_RANGE_ERROR,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            Stop Monitoring Error
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Safety Module LED 코드 및 Safety Module 메뉴얼 참조하여 처리한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_STOP_MONITORING_ERROR,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 상태 에러 \n
            Motor Brake Error
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Safety Module LED 코드 및 Safety Module 메뉴얼 참조하여 처리한다. \n
            모터 상태 점검
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_MOTOR_BRAKE_ERROR,
    /**
    @brief
    @par    설명
            Safety module 에서 발생하는 충돌 감지 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            Safety Module LED 코드 및 Safety Module 메뉴얼 참조하여 처리한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_SAFETY_COLLISION_DETECTION,
    /**
    @brief
    @par    설명
            모터 드라이브의 평균 온도가 90도 이상, 10초 이상 지속될 시 발생
    @par    Sender ID
            로봇 ID
    @par    조치 방법
            로봇이 가열된 상태로 기동 중단. \n
            주변 환경의 온도에 따라 해당 상태에 빠질 수 있다.
    */
    CLINK_EVENT_SUBGRP_ERROR_DRIVE_OVER_TEMPERATURE_SCENARIO,
    /**
    @brief
    @par    설명
            로봇 Friction Estimation 실패
    @par    Sender ID
            로봇 ID
    @par    조치 방법
            Friction Estimation 도 중 일어날 수 있으며 원인은 다양할 수 있다. \n
            해당 이벤트와 함께 일어난 다른 에러 정보를 참조한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_ROBOT_DYNAMICS_CALIBRATION_NOT_MATCHED,
    /**
    @brief
    @par    설명
            현재 EtherCAT 네트워크에서 확인되는 Slave 숫자가 예상과 일치하지 않는 경우
    @par    Sender ID
            0
    @par    조치 방법
            전달되는 실제 Detection 된 Slave 갯수 및 정상적으로 연결되어야 하는 Slave 갯수 정보를 바탕으로 \n
            실제 Detection 된 Slave 갯수를 중심으로 EtherCAT 연결 상태를 확인한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_ACTIVE_ECAT_SLAVE_COUNT_FAULT,
    /**
    @brief
    @par    설명
            EPC Digital input 값이 심하게 흔들리는 경우(10초에 20회 이상)
    @par    Sender ID
            0
    @par    조치 방법
            EPC 에 연결되는 GPIO Port 를 점검한다. \n
            EPC 고장 또는 외부에서 신호를 잘못 주는 경우 발생 가능하다.
    */
    CLINK_EVENT_SUBGRP_ERROR_EPC_DIGITAL_INPUT_ABNORMALITY,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 엔코더와 마그넷 분리시 발생
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC, WELCON Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ENCODER_SIGNAL_AMPLITUDE,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 엔코더와 마그넷 급격히 분리시 발생 (Position이 급격히 달라짐)
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC, WELCON Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ENCODER_POSITION_VALUE,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 엔코더 과전압 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC, WELCON Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ENCODER_OVER_VOLTAGE,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 엔코더 저전압 에러
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC, WELCON Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ENCODER_UNDER_VOLTAGE,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 엔코더 온도 130도 이상시 에러 발생
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC, WELCON Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ENCODER_OVER_TEMPERATURE,
    /**
    @brief
    @par    설명
            [AMC, WELCON] 모터 배터리 2.7V 알람 발생 (배터리 제거시 알람 발생) (2.8V ~ 3.6V 정상동작)
    @par    Sender ID
            EtherCAT Slave ID
    @par    조치 방법
            AMC, WELCON Drive 메뉴얼 참조
    */
    CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ENCODER_LOW_BATTERY,
    /**
    @brief
    @par    설명
            현재 EtherCAT 네트워크에서 확인되는 Slave 구성이 현재 호출된 로봇과 맞지 않음
    @par    Sender ID
            0
    @par    조치 방법
            실제 구성된 로봇과 사용자가 생성한 로봇 모델이 일치하는 지 확인한다.
    */
    CLINK_EVENT_SUBGRP_ERROR_ECAT_SLAVE_CANNOT_MATCH_CURRENT_ROBOT

    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_CURRENT_OFFSET_ERROR,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_CONTINUOUS_OVERLOAD,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_MOTOR_CABLE_OPEN,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_MOTOR_OVER_TEMPERATURE,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ENCODER_COMM_CRC_ERROR,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ENCODER_CABLE_OPEN,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_MOTOR_SETTING_ERROR,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ENCODER_LOW_BATTERY,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_POSITION_TRACKING,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_EXCESSIVE_POSITION_COMMAND,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_PARAMETER_CHECKSUM,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ECAT_COMM_ERROR,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_INVALID_SETTING,
    //CLINK_EVENT_SUBGRP_ERROR_MOTOR_SLAVE_ALIGN_ERROR,

}CLINK_EVENT_SUBGRP_ERROR;

/**
@brief  CLINK_EVENT_GRP_MOTION_COMMAND 와 함께 발생하는 subgroup
*/
typedef enum
{
    CLINK_EVENT_SUBGRP_MOTION_COMMAND_INVALID = CLINK_EVENT_GRP_MOTION_COMMAND + 1,
    /**
    @brief  motion 이 실제 출발하는 순간 발생한다.
            event_sender_id 는 motion command 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_MOTION_COMMAND_STARTED,
    /**
    @brief  blending motion 에서 radius 이하만큼 남았을 때 발생한다.
            event_sender_id 는 motion command 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_MOTION_COMMAND_BLENDING_MOTION_ALMOST_COMPLETED,
    /**
    @brief  motion command 의 profiling (즉, command 생성) \n
            이 완료된 경우 발생한다. \n
            event_sender_id 는 motion command 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_MOTION_COMMAND_PROFILE_COMPLETED,
    /**
    @brief  motion command 의 profiling (즉, command 생성) \n
            이 완료되고, settling 이 완료되면 발생한다. \n
            event_sender_id 는 motion command 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_MOTION_COMMAND_SETTLED_DOWN,
    /**
    @brief  motion command 를 pause 한 경우, \n
            이 때 profiling (즉, command 생성) \n
            이 완료된 경우 발생한다. \n
            event_sender_id 는 motion command 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_MOTION_COMMAND_PUASE_PROFILE_COMPLETED,
    /**
    @brief  motion command 를 pause 한 경우, \n
            이 때 profiling (즉, command 생성) \n
            이 완료되고, settling 이 완료되면 발생한다. \n
            event_sender_id 는 motion command 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_MOTION_COMMAND_PAUSE_SETTLED_DOWN,
    /**
    @brief  motion command 를 stop 한 경우, \n
            이 때 profiling (즉, command 생성) \n
            이 완료된 경우 발생한다. \n
            event_sender_id 는 motion command 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_MOTION_COMMAND_STOP_PROFILE_COMPLETED,
    /**
    @brief  motion command 를 stop 한 경우, \n
            이 때 profiling (즉, command 생성) \n
            이 완료되고, settling 이 완료되면 발생한다. \n
            event_sender_id 는 motion command 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_MOTION_COMMAND_STOP_SETTLED_DOWN,
    /**
    @brief  zigzag weaving motion 실행 중 section change 가 일어날 경우 이벤트가 발생한다. \n
            event_sender_id 는 motion command 의 ID 이다.
            reserved int [0] 배열에 해당 section number 가 기록된다 (0 ~ 7)
    */
    CLINK_EVENT_SUBGRP_MOTION_COMMAND_ZIGZANG_WEAVING_SECTION_CHANGE
}CLINK_EVENT_SUBGRP_MOTION_COMMAND;

/**
@brief  CLINK_EVENT_GRP_ROBOT_SAFETY_VIOLATED 와 함께 발생하는 subgroup
*/
typedef enum
{
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED_INVALID = CLINK_EVENT_GRP_ROBOT_SAFETY_VIOLATED + 1,
    /**
    @brief
    @par    설명
            로봇의 Joint 가 안전설정 범위 이상으로 움직인 경우 발생, 제어SW 에서 감지
    @par    Sender ID
            로봇 ID
    @par    조치 방법
            추가 파라미터로 전달되는 Joint Index 와 초과 각도 값을 확인한다. \n
            안전 설정 해제 - 안전설정 범위 안으로 로봇을 수동 조작 - 안전 설정하여 복귀하며 \n
            모션에 의해 발생한 경우 모션을 수정하도록 한다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED_JOINT_POSITION,
    /**
    @brief
    @par    설명
            로봇의 Joint Speed 가 안전설정 범위를 벗어난 경우 발생, 제어SW 에서 감지
    @par    Sender ID
            로봇 ID
    @par    조치 방법
            추가 파라미터로 전달되는 Joint Index 와 초과 각속도 값을 확인한다. \n
            다시 Servo On 을 하여 로봇을 조작할 수 있으며, \n
            모션에 의해 발생한 경우 모션을 수정하도록 한다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED_JOINT_SPEED,
    /**
    @brief
    @par    설명
            로봇의 Joint Acceleration 이 안전설정 범위를 벗어난 경우 발생, 제어SW 에서 감지
    @par    Sender ID
            로봇 ID
    @par    조치 방법
            추가 파라미터로 전달되는 Joint Index 와 초과 각가속도 값을 확인한다. \n
            다시 Servo On 을 하여 로봇을 조작할 수 있으며, \n
            모션에 의해 발생한 경우 모션을 수정하도록 한다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED_JOINT_ACCELERATION,
    /**
    @brief
    @par    설명
            로봇의 TCP 끝 단이 설정된 안전 평면 밖으로 위치된 경우 발생, 제어SW 에서 감지
    @par    Sender ID
            로봇 ID
    @par    조치 방법
            추가 파라미터로 전달되는 TCP 좌표 값을 확인한다.. \n
            안전 설정 해제 - 안전 평면 설정 범위 안으로 로봇을 수동 조작 - 안전 설정하여 복귀하며 \n
            모션에 의해 발생한 경우 모션을 수정하거나 안전 평면 설정을 변경한다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED_TCP_POSITION,
    /**
    @brief
    @par    설명
            TCP Motion 계열 실행 시 발생 가능성 존재 \n
            로봇의 Inverse Kinematics 의 해가 없는 곳을 진행해야 할 때 발생 \n
            즉, 로봇이 갈 수 없는 위치 지령 시 발생, 제어SW 에서 감지
    @par    Sender ID
            로봇 ID
    @par    조치 방법
            Singularity 가 발생하지 않은 자세를 찾아 모션을 수정한다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED_SINGULARITY,
    /**
    @brief
    @par    설명
            TCP Motion 계열 실행 시 발생 가능성 존재 \n
            모션 프로파일 생성 실패 시 발생 \n
            시작 속도 또는 종료 속도가 존재하는 모션일 경우, \n
            시작/종료 속도에 비해 거리가 짧은 모션일 경우 발생할 수 있음
    @par    Sender ID
            모션 Command ID
    @par    조치 방법
            모션 파라미터를 수정한다. 특히, 시작/종료 속도 및 모션의 총 이동 거리를 고려한다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_MOTION_GENERATION_FAILURE,
    /**
    @brief
    @par    설명
            로봇의 Command(Target) Angle 값과 실제 Actual Angle 값이 일정 차이 이상 \n
            벌어질 경우 발생한다.
    @par    Sender ID
            로봇 ID
    @par    조치 방법
            추가 파라미터로 전달되는 Joint Index 와 Command/Actual Angle 값을 확인한다. \n
            반복적으로 발생 시, 모터 드라이브 튜닝등의 설정을 바꿔야 할 수 있다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED_FOLLOWING_ERROR,
    //CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED_TCP_SPEED,
    //CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED_TCP_ACCELERATION,
    //CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED_OPPOSITE_DIR_WHILE_OUT_OF_SPEC
}CLINK_EVENT_SUBGRP_ROBOT_SAFETY_VIOLATED;

/**
@brief  CLINK_EVENT_GRP_ROBOT_SPEC_VIOLATED 와 함께 발생하는 subgroup
*/
typedef enum
{
    CLINK_EVENT_SUBGRP_ROBOT_SPEC_VIOLATED_INVALID = CLINK_EVENT_GRP_ROBOT_SPEC_VIOLATED + 1,
    /**
    @brief  로봇의 joint 가 기구적 설계 범위 이상으로 움직인 경우 발생한다. \n
            event_sender_id 는 robot 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SPEC_VIOLATED_JOINT_POSITION,
    /**
    @brief  로봇의 joint 속도 가 기구적 설계 범위 이상인 경우 발생한다. \n
            event_sender_id 는 robot 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SPEC_VIOLATED_JOINT_SPEED,
    /**
    @brief  로봇의 joint 가속도 가 기구적 설계 범위 이상인 경우 발생한다. \n
            event_sender_id 는 robot 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SPEC_VIOLATED_JOINT_ACCELERATION,
    /**
    @brief  로봇의 TCP 가 기구적 설계 범위 이상으로 움직인 경우 발생한다. \n
            event_sender_id 는 robot 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SPEC_VIOLATED_TCP_POSITION,
    /**
    @brief  로봇의 TCP 속도 가 기구적 설계 범위 이상인 경우 발생한다. \n
            event_sender_id 는 robot 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SPEC_VIOLATED_TCP_SPEED,
    /**
    @brief  로봇의 TCP 가속도 가 기구적 설계 범위 이상인 경우 발생한다. \n
            event_sender_id 는 robot 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SPEC_VIOLATED_TCP_ACCELERATION
}CLINK_EVENT_SUBGRP_ROBOT_SPEC_VIOLATED;

/**
@brief  CLINK_EVENT_GRP_ROBOT_COLLISION_DETECTED 와 함께 발생하는 subgroup
*/
typedef enum
{
    CLINK_EVENT_SUBGRP_ROBOT_COLLISION_DETECTED_INVALID = CLINK_EVENT_GRP_ROBOT_COLLISION_DETECTED + 1,
    /**
    @brief  로봇의 특정 joint 가 충돌감지된 경우 발생한다.  \n
            event_sender_id 는 robot 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_COLLISION_DETECTED_JOINT,
    /**
    @brief  로봇의 TCP 가 충돌감지된 경우 발생한다.  \n
            event_sender_id 는 robot 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_COLLISION_DETECTED_TCP,
    /**
    @brief  충돌 후, 충돌 완화 기능 사용 시 완화 기능까지 완료되었을 때 발생한다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_COLLISION_MITIGATION_COMPLETED
}CLINK_EVENT_SUBGRP_ROBOT_COLLISION_DETECTED;

/**
@brief  CLINK_EVENT_GRP_ROBOT_REDUNDANT_IO 와 함께 발생하는 subgroup
*/
typedef enum
{
    CLINK_EVENT_SUBGRP_ROBOT_REDUNDANT_IO_INVALID = CLINK_EVENT_GRP_ROBOT_REDUNDANT_IO + 1,
    /**
    @brief  reduced mode 진입 시 발생
    */
    CLINK_EVENT_SUBGRP_ROBOT_REDUNDANT_IO_REDUCED_MODE,
    /**
    @brief  reduced mode 해제 시 발생
    */
    CLINK_EVENT_SUBGRP_ROBOT_REDUNDANT_IO_NORMAL_MODE,
    /**
    @brief  enable switch 기능 진입 시 발생
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_MODULE_REDUNDANT_IO_ENABLE_SWITCH_FUNC_ON,
    /**
    @brief  enable switch 기능 해제 시 발생
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_MODULE_REDUNDANT_IO_ENABLE_SWITCH_FUNC_OFF,
    /**
    @brief  safeguard pause 발생
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_MODULE_REDUNDANT_IO_SAFEGUARD_PAUSE,
    /**
    @brief  safeguard resume 발생
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_MODULE_REDUNDANT_IO_SAFEGUARD_RESUME,
    /**
    @brief
    @par    설명
            설정되어 있는 Redundant IO 에서 두 Digital Input 값이 100ms 이상 다른 값이 들어올 경우 발생 \n
            항상 같은 값이 들어와야 하며 다른 값이 들어온 경우, 신호를 잘못 보냈다고 판단.
    @par    Sender ID
            로봇 ID
    @par    조치 방법
            추가 파라미터로 전달되는 두 IO Index 값의 Redundant IO Digital Input 값을 실제로 확인하여 이상 없는지 점검한다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_REDUNDANT_IO_LINE_VALUE_DIFF,
    /**
    @brief
    @par    설명
            설정되어 있는 Safety Module IO 에서 두 Digital Input 값이 100ms 이상 다른 값이 들어올 경우 발생 \n
            항상 같은 값이 들어와야 하며 다른 값이 들어온 경우, 신호를 잘못 보냈다고 판단.
    @par    Sender ID
            로봇 ID
    @par    조치 방법
            추가 파라미터로 전달되는 두 IO Index 값의 Safety Module IO Digital Input 값을 실제로 확인하여 이상 없는지 점검한다.
    */
    CLINK_EVENT_SUBGRP_ROBOT_SAFETY_MODULE_REDUNDANT_IO_LINE_VALUE_DIFF
}CLINK_EVENT_SUBGRP_ROBOT_REDUNDANT_IO;

/**
@brief  CLINK_EVENT_GRP_CONTROL_BOX_BUTTON_CHANGED 와 함께 발생하는 subgroup
*/
typedef enum
{
    CLINK_EVENT_SUBGRP_CONTROL_BOX_BUTTON_CHANGED_INVALID = CLINK_EVENT_GRP_CONTROL_BOX_BUTTON_CHANGED + 1,
    /**
    @brief  컨트롤 박스 (혹은 티칭 팬던트) 에 연결된 비상정지 버튼이 해제된 경우 발생한다. \n
            event_sender_id 는 control box 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_CONTROL_BOX_BUTTON_CHANGED_EMG_RELEASED,
    /**
    @brief  컨트롤 박스 (혹은 티칭 팬던트) 에 연결된 비상정지 버튼이 선택된 경우 발생한다. \n
            event_sender_id 는 control box 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_CONTROL_BOX_BUTTON_CHANGED_EMG_PUSHED,
    /**
    @brief  컨트롤 박스 (혹은 티칭 팬던트) 에 연결된 전원 버튼이 해제된 경우 발생한다. \n
            event_sender_id 는 control box 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_CONTROL_BOX_BUTTON_CHANGED_PWR_RELEASED,
    /**
    @brief  컨트롤 박스 (혹은 티칭 팬던트) 에 연결된 전원 버튼이 선택된 경우 발생한다. \n
            event_sender_id 는 control box 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_CONTROL_BOX_BUTTON_CHANGED_PWR_PUSHED,
    /**
    @brief  컨트롤 박스 (혹은 티칭 팬던트) 에 연결된 remote 전원 버튼이 해제된 경우 발생한다. \n
            event_sender_id 는 control box 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_CONTROL_BOX_BUTTON_CHANGED_REMOTE_PWR_RELEASED,
    /**
    @brief  컨트롤 박스 (혹은 티칭 팬던트) 에 연결된 remote 전원 버튼이 선택된 경우 발생한다. \n
            event_sender_id 는 control box 의 ID 이다.
    */
    CLINK_EVENT_SUBGRP_CONTROL_BOX_BUTTON_CHANGED_REMOTE_PWR_PUSHED
}CLINK_EVENT_SUBGRP_CONTROL_BOX_BUTTON_CHANGED;

/**
@brief  CLINK_EVENT_GRP_NOTIFICATION 와 함께 발생하는 subgroup
*/
typedef enum
{
    CLINK_EVENT_SUBGRP_NOTIFICATION_INVALID = CLINK_EVENT_GRP_NOTIFICATION + 1,
    /**
    @brief  제어 FW의 시스템 초기화가 끝난 경우 발생한다.
    */
    CLINK_EVENT_SUBGRP_NOTIFICATION_SYSTEM_INITIALIZED,
    /**
    @brief
    @par    설명
            시스템 초기화 실패 시 발생
    @par    Sender ID
            0
    @par    조치 방법
            반복 발생 시, 개발팀에 문의
    */
    CLINK_EVENT_SUBGRP_NOTIFICATION_SYSTEM_FAIL_TO_INITIALIZED,
    /**
    @brief  제어 FW의 EtherCAT 연결이 완료되었을 때 발생(op mode)
    */
    CLINK_EVENT_SUBGRP_NOTIFICATION_ECAT_CONNECTED,
    /**
    @brief  제어 FW의 EtherCAT 연결 해제가 완료되었을 때 발생
    */
    CLINK_EVENT_SUBGRP_NOTIFICATION_ECAT_DISCONNECTED,
    /**
    @brief
    @par    설명
            제어SW 종료 명령 시 발생
    @par    Sender ID
            0
    @par    조치 방법
            정상 종료가 아닌 반복 발생 시, 개발팀에 문의
    */
    CLINK_EVENT_SUBGRP_NOTIFICATION_CONTROL_SW_EXIT,
    /**
    @brief  dynamics calibaration 이 성공적으로 끝났을 때 발생
    */
    CLINK_EVENT_SUBGRP_NOTIFICATION_DYNAMICS_CALIBRATION_COMPLETED,
    /**
    @brief
    @par    설명
            로봇 Friction Estimation 기능 실패 시 발생 \n
            파일 저장 실패 및 예외 발생, 로봇 이상으로 모션 지령 실패 시 발생
    @par    Sender ID
            로봇 ID
    @par    조치 방법
            추가 파라미터로 전달되는 Joint Index 를 참조한다. \n
            모션 실패 시에 부가적으로 나타나는 에러를 분석한다.
    */
    CLINK_EVENT_SUBGRP_NOTIFICATION_DYNAMICS_CALIBRATION_NOT_COMPLETED,
    /**
    @brief  제어권을 획득했을 때 발생
    */
    CLINK_EVENT_SUBGRP_NOTIFICATION_CONTROL_TAKE,
    /**
    @brief  제어권이 넘어갔을 때 발생
    */
    CLINK_EVENT_SUBGRP_NOTIFICATION_CONTROL_RELEASE,
    /**
    @brief
    @par    설명
            로봇의 Recording 기능 이용 시 제한 시간(10분)을 초과 했을 때 발생
    @par    Sender ID
            0
    @par    조치 방법
            Recording 기능은 10분 이내에서 기능 이용
    */
    CLINK_EVENT_SUBGRP_NOTIFICATION_ROBOT_RECORDING_TIME_OUT,
    /**
    @brief  로봇 생성 시, FT Sensor 장착 여부에 대한 정보를 전달한다.
    */
    CLINK_EVENT_SUBGRP_NOTIFICATION_FT_SENSOR_STATUS
}CLINK_EVENT_SUBGRP_NOTIFICATION;

/**
@brief  CLINK_EVENT_GRP_MODBUS_NOTI 와 함께 발생하는 subgroup
*/
typedef enum
{
    CLINK_EVENT_SUBGRP_MODBUS_NOTI_INVALID = CLINK_EVENT_GRP_MODBUS_NOTI + 1,
    /**
    @brief  UI 프로그램 시작 실행
    */
    CLINK_EVENT_SUBGRP_MODBUS_NOTI_START_PROGRAM,
    /**
    @brief  실행중인 프로그램 일시정지
    */
    CLINK_EVENT_SUBGRP_MODBUS_NOTI_PAUSE_PROGRAM,
    /**
    @brief  실행중인 프로그램 정지
    */
    CLINK_EVENT_SUBGRP_MODBUS_NOTI_STOP_PROGRAM,
    /**
    @brief  로봇 동작 중 & 용접기 에러 이벤트 발생
    */
    CLINK_EVENT_SUBGRP_MODBUS_NOTI_WELDER_ERROR
}CLINK_EVENT_SUBGRP_MODBUS_NOTI;

/**
@brief  CLINK-API 의 기능 중 binary state 를 가지는 \n
        기능의 설정에 사용한다.
*/
typedef enum
{
    /**
    @brief  특정 기능의 ON (enabled, activated) 상태를 의미한다.
    */
    CLINK_SWITCH_ON = 400100,
    /**
    @brief  특정 기능의 OFF (disabled, deactivated) 상태를 의미한다.
    */
    CLINK_SWITCH_OFF
}CLINK_SWITCH;

/**
@brief  EtherCAT 연결 상태를 나타낸다.
*/
typedef enum
{
    /**
    @brief  연결 시도 중
    */
    CLINK_ECAT_CONN_STATE_CONNECT_TRYING = CLINK_SWITCH_ON + 100,
    /**
    @brief  연결 완료된 상태
    */
    CLINK_ECAT_CONN_STATE_CONNECTED,
    /**
    @brief  종료 시도 중
    */
    CLINK_ECAT_CONN_STATE_DISCONNECT_TRYING,
    /**
    @brief  종료된 상태
    */
    CLINK_ECAT_CONN_STATE_DISCONNECTED
}CLINK_ECAT_CONN_STATE;

/**
@brief  CLINK-API 의 기능 중 회전 방향을 설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  시계방향
    */
    CLINK_ROTATION_DIRECTION_CLOCK_WISE = CLINK_ECAT_CONN_STATE_CONNECT_TRYING + 100,
    /**
    @brief  반시계방향
    */
    CLINK_ROTATION_DIRECTION_COUNTER_CLOCK_WISE,
}CLINK_ROTATION_DIRECTION;

/**
@brief  CLINK-API 의 기능 중 진행 방향을 설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  양의 방향
    */
    CLINK_DIRECTION_POSITIVE = CLINK_ROTATION_DIRECTION_CLOCK_WISE + 100,
    
    /**
    @brief  음의 방향
    */
    CLINK_DIRECTION_NEGATIVE

}CLINK_DIRECTION;

/**
@brief  모션의 속도 profile 을 하는 방법을 설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  가속구간과 감속구간의 기울기가 비대칭이면서 \n
            속도의 변화가 부드러운 (즉, 불연속구간이 없는) \n
            S Curve 프로파일을 생성한다.
    @remark 현재 기본 프로파일이다.
    */
    CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE = CLINK_DIRECTION_POSITIVE + 100,
    /**
    @brief  일반적인 사다리꼴의 프로파일을 생성한다.
    @remark 현재 지원하지 않음.
    */
    CLINK_VELOCITY_PROFILE_TRAPEZODAL
}CLINK_VELOCITY_PROFILE;

/**
@brief  모션의 위치를 보간하는 방법을 설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  직선 보간
    */
    CLINK_INTERPOLATOR_LINEAR = CLINK_VELOCITY_PROFILE_ASYMMETRIC_S_CURVE + 100,
    /**
    @brief  시작 위치 -> 중간 위치 -> 종료 위치 \n
            의 내접원을 사용하여 보간한다.
    */
    CLINK_INTERPOLATOR_CIRCULAR,
    /**
    @brief  시작 위치 -> 중간 위치 -> 종료 위치 \n
            의 내접원을 사용하여 보간한다.
            원호가 끝나는 부분으로 도착위치를 바꾼다. \n
            해당 기능은 현재 지원하지 않음
    */
    CLINK_INTERPOLATOR_CIRCULAR_TO_TANGENT,
    /**
    @brief  시작 위치 -> 중간 위치 -> 종료 위치 \n
            의 외접원을 사용하여 보간한다.
    */
    CLINK_INTERPOLATOR_ARC,
    /**
    @brief  시작 위치 -> 종료 위치 \n
            를 완벽한 구를 그리는 보간을 한다. \n
            중간 위치는 평면을 구성하기 위해 사용한다.
    */
    CLINK_INTERPOLATOR_CIRCLE,
    /**
    @brief  tcp 끝 단의 위치는 고정한채 orientation의 변화만 주는 모션
    */
    CLINK_INTERPOLATOR_ORT_JOG,
    /**
    @brief  입력한 포인트들을 따라 곡선 보간한다
    */
    CLINK_INTERPOLATOR_FREE_PATH
}CLINK_INTERPOLATOR;

/**
@brief  아날로그 IO 의 모드를 설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  전압 모드
    */
    CLINK_ANALOG_IO_MODE_VOLTAGE = CLINK_INTERPOLATOR_LINEAR + 100,
    /**
    @brief  전류 모드
    */
    CLINK_ANALOG_IO_MODE_CURRENT
}CLINK_ANALOG_IO_MODE;

/**
@brief  Tool I/O 에 대해 Power voltage option 을 설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  Tool I/O 에 대한 Power voltage option이 0V
    */
    CLINK_TOOL_POWER_VOLTAGE_OPTION_0 = CLINK_ANALOG_IO_MODE_VOLTAGE + 100,
    /**
    @brief  Tool I/O 에 대한 Power voltage option이 12V
    */
    CLINK_TOOL_POWER_VOLTAGE_OPTION_12 = CLINK_TOOL_POWER_VOLTAGE_OPTION_0 + 12,
    /**
    @brief  Tool I/O 에 대한 Power voltage option이 24V
    */
    CLINK_TOOL_POWER_VOLTAGE_OPTION_24 = CLINK_TOOL_POWER_VOLTAGE_OPTION_0 + 24
}CLINK_TOOL_IO_POWER_VOLTAGE_OPTION;

/**
@brief  로봇의 상태를 의미한다.
*/
typedef enum
{
    /**
    @brief  로봇에 지령된 명령이 없고, \n
            움직이지 않는 상태
    */
    CLINK_MOVING_STATE_IDLE = CLINK_TOOL_POWER_VOLTAGE_OPTION_0 + 100,
    /**
    @brief  로봇이 현재 어떤 명령을 실행하여 \n
            움직이고 있는 상태
    */
    CLINK_MOVING_STATE_MOVING,
    /**
    @brief  로봇이 pause 명령을 실행하며 \n
            멈추고 있는 상태 \n
            (즉, 아직 완벽히 정지하지는 않은 상태)
    */
    CLINK_MOVING_STATE_PAUSING,
    /**
    @brief  로봇이 pause 명령 실행을 종료하여 \n
            현재 멈추어 있는 상태
    */
    CLINK_MOVING_STATE_PAUSED,
    /**
    @brief  로봇이 stop 명령을 실행하며 \n
            멈추고 있는 상태 \n
            (즉, 아직 완벽히 정지하지는 않은 상태)
    */
    CLINK_MOVING_STATE_STOPPING,
    /**
    @brief  로봇이 stop 명령 실행을 종료하여 \n
            현재 멈추어 있는 상태
    */
    CLINK_MOVING_STATE_STOPPED,
    /**
    @brief  로봇이 응급정지 명령을 실행하며 \n
            멈추고 있는 상태 \n
            (즉, 아직 완벽히 정지하지는 않은 상태)
    */
    CLINK_MOVING_STATE_EMG_STOPPING,
    /**
    @brief  로봇이 응급 정지 명령 실행을 종료하여 \n
            현재 멈추어 있는 상태
    */
    CLINK_MOVING_STATE_EMG_STOPPED,
    /**
    @brief  Invalid 상태. 초기화 등에 사용
    */
    CLINK_MOVING_STATE_INVALID
}CLINK_MOVING_STATE;

/**
@brief  로봇이 현재 safety limit 을 위반한 \n
        상태인지 여부를 의미한다.
*/
typedef enum
{
    /**
    @brief  로봇이 현재 safety limit 을 위반하지 않은 상태
    */
    CLINK_ROBOT_SAFETY_STATE_SAFE = CLINK_MOVING_STATE_IDLE + 100,
    /**
    @brief  로봇이 현재 safety limit 을 위반한 상태
    */
    CLINK_ROBOT_SAFETY_STATE_NOT_SAFE
}CLINK_ROBOT_SAFETY_STATE;

/**
@brief  버튼의 상태를 의미한다.
*/
typedef enum
{
    /**
    @brief  버튼이 눌린 상태
    */
    CLINK_BUTTON_STATE_PUSHED = CLINK_ROBOT_SAFETY_STATE_SAFE + 100,
    /**
    @brief  버튼이 눌려지지 않은 상태
    */
    CLINK_BUTTON_STATE_RELEASED
}CLINK_BUTTON_STATE;

/**
@brief  control box - 로봇 간 fieldbus (네트워크 통신) \n
        의 상태를 의미한다.
*/
typedef enum
{
    /**
    @brief  연결된 상태
    */
    CLINK_FIELD_BUS_STATE_CONNECTED = CLINK_BUTTON_STATE_PUSHED + 100,
    /**
    @brief  연결을 시도 중인 상태
    */
    CLINK_FIELD_BUS_STATE_CONNECTING,
    /**
    @brief  연결을 끊어진 상태
    */
    CLINK_FIELD_BUS_STATE_DISCONNECTED
}CLINK_FIELD_BUS_STATE;

/**
@brief  로봇의 설치 면을 설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  바닥면에 설치한 상태
    */
    CLINK_INSTALLATION_MOUNT_FLOOR = CLINK_FIELD_BUS_STATE_CONNECTED + 100,
    /**
    @brief  벽면에 설치한 상태
    */
    CLINK_INSTALLATION_MOUNT_WALL,
    /**
    @brief  천장에 설치한 상태
    */
    CLINK_INSTALLATION_MOUNT_CEILING
}CLINK_INSTALLATION_MOUNT;

/**
@brief  로봇의 TCP 혹은 툴을 보호하기 위한 \n
        안전영역을 특정 shape 으로 표현하여 \n
        설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  박스 형태의 shape
    */
    CLINK_BOUNDING_SHAPE_BOX = CLINK_INSTALLATION_MOUNT_FLOOR + 100,
    /**
    @brief  구 형태의 shape
    */
    CLINK_BOUNDING_SHAPE_SPHERE,
    /**
    @brief  원기둥 형태의 shape
    */
    CLINK_BOUNDING_SHAPE_CYLINDER,
    /**
    @brief  원뿔 형태의 shape
    */
    CLINK_BOUNDING_SHAPE_CONE
}CLINK_BOUNDING_SHAPE;

/**
@brief  로봇 모델의 identifier
*/
typedef enum
{
    /**
    @brief  NONE
    */
    CLINK_ROBOT_MODEL_NONE = CLINK_BOUNDING_SHAPE_BOX + 100,
    /**
    @brief  User Defined Robot 01
    */
    CLINK_ROBOT_MODEL_USERROBOT01,
    /**
    @brief  User Defined Robot 02
    */
    CLINK_ROBOT_MODEL_USERROBOT02,
    /**
    @brief  User Defined Collaborative Robot 01
    */
    CLINK_ROBOT_MODEL_USERCOROBOT01,
    /**
    @brief  User Defined Collaborative Robot 02
    */
    CLINK_ROBOT_MODEL_USERCOROBOT02,
    /**
    @brief  Teleoperated Robot
    */
    CLINK_ROBOT_MODEL_TELEOPERATEDROBOT,

    /**
    @brief  3세대 OHT
    */
    CLINK_ROBOT_MODEL_OHT,
    /**
    @brief  3세대 AMR350
    */
    CLINK_ROBOT_MODEL_AMR350,
    /**
    @brief  3세대 AMR1500
    */
    CLINK_ROBOT_MODEL_AMR1500,

    /**
    @brief  아래 enum 에는 HCR Series 로봇만 위치시킨다.
            또한 신규 추가 로봇 모델은 무조건 맨 아래에 위치시키도록 한다.
    */
    CLINK_ROBOT_MODEL_DONOT_USE_HCR_BELOW,
    /**
    @brief  HCR3
    */
    CLINK_ROBOT_MODEL_HCR3,
    /**
    @brief  HCR5
    */
    CLINK_ROBOT_MODEL_HCR5,
    /**
    @brief  HCR12
    */
    CLINK_ROBOT_MODEL_HCR12,
    /**
    @brief  3세대 HCR14
    */
    CLINK_ROBOT_MODEL_HCR14,
    /**
    @brief  3세대 HCR10L
    */
    CLINK_ROBOT_MODEL_HCR10L,
    /**
    @brief  3세대 HCR5W
    */
    CLINK_ROBOT_MODEL_HCR5W,
    /**
    @brief  3세대 HCR5WA
    */
    CLINK_ROBOT_MODEL_HCR5WA,
    /**
    @brief  3세대 HCR32
    */
    CLINK_ROBOT_MODEL_HCR32,
    /**
    @brief  HCR5WB
    */
    CLINK_ROBOT_MODEL_HCR5WB
}CLINK_ROBOT_MODEL;

/**
@brief  제어박스 모델의 identifier
*/
typedef enum
{
    /**
    @brief  NONE
    */
    CLINK_CBOX_MODEL_NONE = CLINK_ROBOT_MODEL_NONE + 100,
    /**
    @brief  임의의 EtherCAT IO 만 가지는 CBOX
    */
    CLINK_CBOX_MODEL_EMPTY,
    /**
    @brief  임의의 EtherCAT IO 만 가지는 시뮬레이션 CBOX
    */
    CLINK_CBOX_MODEL_EMPTY_SIM,
    /**
    @brief  3세대 제어박스 for HCR : ARK-3531
    */
    CLINK_CBOX_MODEL_3GEN,
    /**
    @brief  3세대 제어박스 for AMR : AFE-R360
    */
    CLINK_CBOX_MODEL_3GEN_AMR,
    /**
    @brief  HCR5 용 소프트웨어 시뮬레이션 제어박스
    */
    CLINK_CBOX_MODEL_3GEN_SIM
}CLINK_CBOX_MODEL;

/**
@brief  TCP 의 위치에 대한 좌표계를 의미한다.
*/
typedef enum
{
    /**
    @brief  로봇의 원점을 좌표계의 원점으로 \n
            설정한 기본 좌표계
    */
    CLINK_REF_COORDINATE_BASE = CLINK_CBOX_MODEL_NONE + 100,
    /**
    @brief  현 TCP 의 위치 및 방향을 \n
            각각 좌표계의 원점 및 축방향으로 \n
            설정한 좌표계
    */
    CLINK_REF_COORDINATE_TCP
}CLINK_REF_COORDINATE;

/**
@brief  입력 또는 출력된 pose 정보가 flange 위치인지 tcp 위치인지 정의한다.
*/
typedef enum
{
    /**
    @brief  입력 또는 출력된 pose의 위치가 flange 단의 위치이다.
    */
    CLINK_POSE_TYPE_FLANGE = CLINK_REF_COORDINATE_BASE + 100,
    /**
    @brief  입력 또는 출력된 pose의 위치가 tcp 단의 위치이다.
    */
    CLINK_POSE_TYPE_TCP
}CLINK_POSE_TYPE;

/**
@brief  Recording and Playback 기능을 위한 \n
        interpolation 방법을 설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  TCP 의 방향 및 위치를 기준으로 보간한다.
    @remark 현재 이 옵션만 사용가능
    */
    CLINK_RECORDING_INTERPOLATION_TCP = CLINK_POSE_TYPE_FLANGE + 100,
    /**
    @brief  각 Joint 별로 보간한다.
    @remark 현재 지원하지 않음.
    */
    CLINK_RECORDING_INTERPOLATION_JOINT
}CLINK_RECORDING_INTERPOLATION;

/**
@brief  Recording and Playback 기능을 위한 \n
        interpolation 방법을 설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  Weaving 패턴 중 완벽한 ZIGZAG
    */
    CLINK_WELDING_WEAVING_PATTERN_ZIGZAG = CLINK_RECORDING_INTERPOLATION_TCP + 100,
    /**
    @brief  Weaving 패턴 중 SINE WAVE 형태 \n
            ZIGZAG 와 비슷하게 상하 (혹은 좌우) 로 \n
            이동하지만, 그 끝점을 둥글게 이동
    */
    CLINK_WELDING_WEAVING_PATTERN_SINE_WAVE,
    /**
    @brief  Weaving 패턴 중 ELLIPSE 형태 \n
            시계 또는 반시계 방향으로 원을 그리며 이동
    */
    CLINK_WELDING_WEAVING_PATTERN_ELLIPSE

}CLINK_WELDING_WEAVING_PATTERN;

/**
@brief Arc sensing data 파일을 다운 받은 후, 새로운 Weaving motion에  Arc sensing offset 값을 적용할 때 motion의 길이가 다름에 따라 생기는 처리방법
*/
typedef enum
{
    /**
    @brief  Arc sensing 기록된 motion의 중간 구간을 이용하여 Arc sensing offset 값을 반영
    */
    CLINK_ARC_SENSING_PLAYBACK_MODE_MIDDLE = CLINK_WELDING_WEAVING_PATTERN_ZIGZAG + 100,
    /**
    @brief  전체 motion의 비율만큼 지났을 때 부터 Arc sensing offset 값을 반영
    */
    CLINK_ARC_SENSING_PLAYBACK_MODE_START_PERCENT
}CLINK_ARC_SENSING_PLAYBACK_MODE;

/**
@brief  이중 IO 기능 정의
*/
typedef enum
{
    /**
    @brief  지정된 두 IO 중 한 개가 high (1) 인 경우 (OR 조건), \n
            emergency button 이 눌린 것과 동일하게 동작 \n
            현재 지원하지 않음
    */
    CLINK_CBOX_IO_REDUNDANT_IN_EMERGENCY_STOP = CLINK_ARC_SENSING_PLAYBACK_MODE_MIDDLE + 100,
    /**
    @brief  지정된 두 IO가 모두 low (0) 인 경우 (OR 조건), \n
            reduced mode 로 동작
    */
    CLINK_CBOX_IO_REDUNDANT_IN_REDUCED_MODE,
    /**
    @brief  지정된 두 IO가 low-high(100ms 이상 지속)-low 가 인지되었을 때 \n
            safeguard IO pause resume 기능이 resume 가능 상태일 때 resume 호출
    */
    CLINK_CBOX_IO_REDUNDANT_IN_SAFEGUARD_RESET,
    /**
    @brief  지정된 두 IO 에 대해 emergency stop 상태인 경우, low (0) 을 출력, \n
            그 외, high (1) 출력
    */
    CLINK_CBOX_IO_REDUNDANT_OUT_EMERGENCY_STOP,
    /**
    @brief  지정된 두 IO 에 대해 로봇이 움직이는 경우, low (0) 을 출력, \n
            그 외, high (1) 출력
    */
    CLINK_CBOX_IO_REDUNDANT_OUT_ROBOT_MOVING,
    /**
    @brief  Stop 명령이 지령된 후 stop 프로파일링이 완료되는 시점까지, high (1) 을 출력, \n
            그 외, low (0) 출력
    */
    CLINK_CBOX_IO_REDUNDANT_OUT_ROBOT_NOT_STOPPING,
    /**
    @brief  충돌감지가 된 경우, high (1) 출력 \n
            충돌감지가 되지 않은 경우, low (0) 출력
    */
    CLINK_CBOX_IO_REDUNDANT_OUT_COLLISION_DETECTED,
    /**
    @brief  reduced mode 인 경우, low (0) 을 출력, \n
            그 외, high (1) 출력
    */
    CLINK_CBOX_IO_REDUNDANT_OUT_REDUCED_MODE,
    /**
    @brief  CBOX_REDUNDANT_IO_OUT_REDUCED_MODE 의 inverse
    */
    CLINK_CBOX_IO_REDUNDANT_OUT_NOT_REDUCED_MODE,
    /**
    @brief  어떤 기능도 할당되어 있지 않은 상태
    */
    CLINK_CBOX_IO_REDUNDANT_NOT_ASSIGNED
}CLINK_CBOX_IO_REDUNDANT;

/**
@brief  safeguard IO 기능 정의
*/
typedef enum
{
    /**
    @brief  지정된 두 IO가 모두 low (0) 인 경우 (OR 조건), \n
            pause 실행, 벗어나는 경우 resume 실행
    */
    CLINK_SAFETY_MODULE_IO_REDUNDANT_IN_PAUSE_RESUME = CLINK_CBOX_IO_REDUNDANT_IN_EMERGENCY_STOP + 100,
    /**
    @brief  Enable Switch가 Active한 상태에서, Direct teaching 및 모든 모션 지령 금지
    */
    CLINK_SAFETY_MODULE_IO_REDUNDANT_IN_ENABLE_SWITCH,
    /**
    @brief  어떤 기능도 할당되어 있지 않은 상태 \n
    */
    CLINK_SAFETY_MODULE_IO_REDUNDANT_NOT_ASSIGNED
}CLINK_SAFETY_MODULE_IO_REDUNDANT;

/**
@brief  IO 종류를 설정한다.
*/
typedef enum
{
    /**
    @brief  IO module digital input in control box
    */
    CLINK_IO_TYPE_CBOX_DIGITAL_INPUT = CLINK_SAFETY_MODULE_IO_REDUNDANT_IN_PAUSE_RESUME + 100,
    /**
    @brief  IO module digital output in control box
    */
    CLINK_IO_TYPE_CBOX_DIGITAL_OUTPUT,
    /**
    @brief  IO module analog input in control box
    */
    CLINK_IO_TYPE_CBOX_ANALOG_INPUT,
    /**
    @brief  IO module analog input in control box
    */
    CLINK_IO_TYPE_CBOX_ANALOG_OUTPUT,
    /**
    @brief  safety module IO module digital input in control box
    */
    CLINK_IO_TYPE_CBOX_SAFETY_MODULE_DIGITAL_INPUT = CLINK_IO_TYPE_CBOX_DIGITAL_INPUT + 10,
    /**
    @brief  safety module IO module digital output in control box
    */
    CLINK_IO_TYPE_CBOX_SAFETY_MODULE_DIGITAL_OUTPUT,
    /**
    @brief  safety module IO module analog input in control box
    */
    CLINK_IO_TYPE_CBOX_SAFETY_MODULE_ANALOG_INPUT,
    /**
    @brief  safety module IO module analog output in control box
    */
    CLINK_IO_TYPE_CBOX_SAFETY_MODULE_ANALOG_OUTPUT,
    /**
    @brief  tool IO module digital input in robot
    */
    CLINK_IO_TYPE_TOOL_DIGITAL_INPUT = CLINK_IO_TYPE_CBOX_DIGITAL_INPUT + 20,
    /**
    @brief  tool IO module digital output in robot
    */
    CLINK_IO_TYPE_TOOL_DIGITAL_OUTPUT,
    /**
    @brief  tool IO module analog input in robot
    */
    CLINK_IO_TYPE_TOOL_ANALOG_INPUT,
    /**
    @brief  tool IO module analog output in robot
    */
    CLINK_IO_TYPE_TOOL_ANALOG_OUTPUT,
} CLINK_IO_TYPE;

/**
@brief  IO control condition
*/
typedef enum
{
    CLINK_IO_CONTROL_CONDITION_INVALID = CLINK_IO_TYPE_CBOX_DIGITAL_INPUT + 100,
    CLINK_IO_CONTROL_CONDITION_BEGIN,
    /**
    @brief  모션이 진행한 시간이 지났을 때 digital output setting
    */
    CLINK_IO_CONTROL_CONDITION_TIME,
    /**
    @brief  모션이 진행한 percentage 초과 시 digital output setting
    */
    CLINK_IO_CONTROL_CONDITION_PERCENTAGE,
    CLINK_IO_CONTROL_CONDITION_END
} CLINK_IO_CONTROL_CONDITION;

/**
@brief  orientation 보간 방법을 지정한다. 기본 지정자는 SLERP 이다.
*/
typedef enum
{
    /**
    @brief  Slerp algorithm을 이용하여 orientation 보간을 실시한다.
    */
    CLINK_ORIENTATION_MODE_SLERP = CLINK_IO_CONTROL_CONDITION_INVALID + 100,
    /**
    @brief  Axis angle 을 사용하여 기준축을 중심으로 회전하는 orientation 보간을 실시한다.
    */
    CLINK_ORIENTATION_MODE_AXIS,
    /**
    @brief  사용자가 입력한 target orientation은 무시되고 자동으로 모션에 맞는 orientation 보간을 실시한다. \n
            원호를 그리는 구간에 있어 중심축을 회전하는 보간을 실시하며 그 외에는 일정하게 유지된다. \n
            즉, target orientation 은 start orientation 에 따라 계산되며 사용자가 입력한 orientation과는 무관하다. \n
            CLINK_INTERPOLATOR_LINEAR: orientation은 변화없이 일정하게 유지된다. 
            그 외 : 중심 축을 기준으로 회전한다.(중심 축 기준 tcp orientation jog 변화와 유사)
    */
    CLINK_ORIENTATION_MODE_AUTO
}CLINK_ORIENTATION_MODE;

/**
@brief  robot의 conveyor mode ON/OFF 상태 표현
*/
typedef enum
{
    /**
    @brief  conveyor mode OFF 상태
    */
    CLINK_CONVEYOR_MODE_OFF = CLINK_ORIENTATION_MODE_SLERP + 100,
    /**
    @brief  conveyor mode ON 상태
    */
    CLINK_CONVEYOR_MODE_ON
}CLINK_CONVEYOR_MODE;

/**
@brief  conveyor encoder type
*/
typedef enum
{
    /**
    @brief  quadrature type encoder
    */
    CLINK_CONVEYOR_ENCODER_TYPE_QUADRATURE = CLINK_CONVEYOR_MODE_OFF + 100,
    /**
    @brief  count type encoder
    */
    CLINK_CONVEYOR_ENCODER_TYPE_COUNT
}CLINK_CONVEYOR_ENCODER_TYPE;

/**
@brief  로봇 inverse kinematics 계산에 사용되는 Input Posture
*/
typedef enum _CLINK_ROBOT_POSTURE
{
    /**
    @brief  front / above / flip 에 해당하는 해
    */
    CLINK_ROBOT_POSTURE_FRONT_ABOVE_FLIP = CLINK_CONVEYOR_ENCODER_TYPE_QUADRATURE + 100,
    /**
    @brief  front / above / nonflip 에 해당하는 해
    */
    CLINK_ROBOT_POSTURE_FRONT_ABOVE_NONFLIP,
    /**
    @brief  front / below / flip 에 해당하는 해
    */
    CLINK_ROBOT_POSTURE_FRONT_BELOW_FLIP,
    /**
    @brief  front / below / nonflip 에 해당하는 해
    */
    CLINK_ROBOT_POSTURE_FRONT_BELOW_NONFLIP,
    /**
    @brief  rear / above / flip 에 해당하는 해
    */
    CLINK_ROBOT_POSTURE_REAR_ABOVE_FLIP,
    /**
    @brief  rear / above / nonflip 에 해당하는 해
    */
    CLINK_ROBOT_POSTURE_REAR_ABOVE_NONFLIP,
    /**
    @brief  rear / below / flip 에 해당하는 해
    */
    CLINK_ROBOT_POSTURE_REAR_BELOW_FLIP,
    /**
    @brief  rear / below / nonflip 에 해당하는 해
    */
    CLINK_ROBOT_POSTURE_REAR_BELOW_NONFLIP
} CLINK_ROBOT_POSTURE;

/**
@brief  Fieldbus (EtherCAT) 의 네트워크의 모드를 설정할 때 사용한다.
*/
typedef enum
{
    CLINK_OPERATIONAL_MODE_BEGIN = CLINK_ROBOT_POSTURE_FRONT_ABOVE_FLIP + 100,
    /**
    @brief  Init 모드
    */
    CLINK_OPERATIONAL_MODE_INIT = CLINK_OPERATIONAL_MODE_BEGIN + 1,
    /**
    @brief  Pre OP 모드
    */
    CLINK_OPERATIONAL_MODE_PRE_OPERATIONAL = CLINK_OPERATIONAL_MODE_BEGIN + 2,
    /**
    @brief  Bootstrap 모드
    */
    CLINK_OPERATIONAL_MODE_BOOTSTRAP = CLINK_OPERATIONAL_MODE_BEGIN + 3,
    /**
    @brief  Safe OP 모드
    */
    CLINK_OPERATIONAL_MODE_SAFE_OPERATIONAL = CLINK_OPERATIONAL_MODE_BEGIN + 4,
    /**
    @brief  OP 모드
    */
    CLINK_OPERATIONAL_MODE_OPERATIONAL = CLINK_OPERATIONAL_MODE_BEGIN + 8,
}CLINK_OPERATIONAL_MODE;

/**
@brief  EtherCAT Motor Drive의 control 모드
*/
typedef enum
{
    CLINK_DRIVE_CONTROL_MODE_BEGIN = CLINK_OPERATIONAL_MODE_BEGIN + 100,
    /**
    @brief  속도 기반 control 모드
    */
    CLINK_DRIVE_CONTROL_MODE_VELOCITY_PROFILE = CLINK_DRIVE_CONTROL_MODE_BEGIN + 3,
    /**
    @brief  Home 위치 설정 시
    */
    CLINK_DRIVE_CONTROL_MODE_HOMING = CLINK_DRIVE_CONTROL_MODE_BEGIN + 6,
    /**
    @brief  위치 기반 control 모드
    */
    CLINK_DRIVE_CONTROL_MODE_POSITION = CLINK_DRIVE_CONTROL_MODE_BEGIN + 8,
    /**
    @brief  속도 기반 control 모드
    */
    CLINK_DRIVE_CONTROL_MODE_VELOCITY = CLINK_DRIVE_CONTROL_MODE_BEGIN + 9,
    /**
    @brief  torque 기반 control 모드
    */
    CLINK_DRIVE_CONTROL_MODE_TORQUE = CLINK_DRIVE_CONTROL_MODE_BEGIN + 10
}CLINK_DRIVE_CONTROL_MODE;

/**
@brief  Force Torque Sensor가 사용하는 Filter 종류를 정의한다.
*/
typedef enum
{
    /**
    @brief  no filter
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_NO_FILTER = CLINK_DRIVE_CONTROL_MODE_BEGIN + 100,
    /**
    @brief  filter 1
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_1,
    /**
    @brief  filter 2
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_2,
    /**
    @brief  filter 3
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_3,
    /**
    @brief  filter 4
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_4,
    /**
    @brief  filter 5
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_5,
    /**
    @brief  filter 6
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_6,
    /**
    @brief  filter 7
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_7,
    /**
    @brief  filter 8
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_8
}CLINK_FORCE_TORQUE_SENSOR_FILTER;

/**
@brief  Force Torque Sensor가 동작하는 Sample Rate를 정의한다.
*/
typedef enum
{
    /**
    @brief  487Hz
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_SAMPLE_RATE_487HZ = CLINK_FORCE_TORQUE_SENSOR_FILTER_NO_FILTER + 100,
    /**
    @brief  975Hz
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_SAMPLE_RATE_975HZ,
    /**
    @brief  1990Hz
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_SAMPLE_RATE_1990HZ,
    /**
    @brief  3900Hz
    */
    CLINK_FORCE_TORQUE_SENSOR_FILTER_SAMPLE_RATE_3900HZ
}CLINK_FORCE_TORQUE_SENSOR_SAMPLE_RATE;

/**
@brief  Force Torque Sensor를 Calibration 한다.
*/
typedef enum
{
    /**
    @brief  Range 0
    */
    CLINK_FORCE_TORQUE_SENSOR_CALIBRATION_RANGE_0 = CLINK_FORCE_TORQUE_SENSOR_FILTER_SAMPLE_RATE_487HZ + 100,
    /**
    @brief  Range 1
    */
    CLINK_FORCE_TORQUE_SENSOR_CALIBRATION_RANGE_1
}CLINK_FORCE_TORQUE_SENSOR_CALIBRATION;

/**
@brief  모드버스 IO Type 설정
*/
typedef enum
{
    CLINK_MODBUS_CLIENT_IO_TYPE_BEGIN = CLINK_FORCE_TORQUE_SENSOR_CALIBRATION_RANGE_0 + 100,
    /**
    @brief  Digital input
    */
    CLINK_MODBUS_CLIENT_IO_TYPE_DIGITAL_INPUT,
    /**
    @brief  Digital output
    */
    CLINK_MODBUS_CLIENT_IO_TYPE_DIGITAL_OUTPUT,
    /**
    @brief  Register input
    */
    CLINK_MODBUS_CLIENT_IO_TYPE_REGISTER_INPUT,
    /**
    @brief  Register output
    */
    CLINK_MODBUS_CLIENT_IO_TYPE_REGISTER_OUTPUT
} CLINK_MODBUS_CLIENT_IO_TYPE;

/**
@brief  모드버스 IO FREQUENCY 설정
*/
typedef enum
{
    CLINK_MODBUS_CLIENT_IO_UPDATE_HZ_BEGIN = CLINK_MODBUS_CLIENT_IO_TYPE_BEGIN + 100,
    /**
    @brief  Once
    */
    CLINK_MODBUS_CLIENT_IO_UPDATE_HZ_0,
    /**
    @brief  1 Hz
    */
    CLINK_MODBUS_CLIENT_IO_UPDATE_HZ_1,
    /**
    @brief  2 Hz
    */
    CLINK_MODBUS_CLIENT_IO_UPDATE_HZ_2,
    /**
    @brief  5 Hz
    */
    CLINK_MODBUS_CLIENT_IO_UPDATE_HZ_5,
    /**
    @brief  10 Hz
    */
    CLINK_MODBUS_CLIENT_IO_UPDATE_HZ_10,
    /**
    @brief  20 Hz
    */
    CLINK_MODBUS_CLIENT_IO_UPDATE_HZ_20,
    /**
    @brief  50 Hz
    */
    CLINK_MODBUS_CLIENT_IO_UPDATE_HZ_50,
    /**
    @brief  100 Hz
    */
    CLINK_MODBUS_CLIENT_IO_UPDATE_HZ_100
} CLINK_MODBUS_CLIENT_IO_UPDATE_HZ;

/**
@brief  모드버스 client 상태
*/
typedef enum
{
    CLINK_MODBUS_CLIENT_STATUS_BEGIN = CLINK_MODBUS_CLIENT_IO_UPDATE_HZ_BEGIN + 100,
    /**
    @brief  모드버스 client Not connect
    */
    CLINK_MODBUS_CLIENT_STATUS_NOT_CONNECT,
    /**
    @brief  모드버스 client Normal
    */
    CLINK_MODBUS_CLIENT_STATUS_NORMAL,
    /**
    @brief  모드버스 client Device busy
    */
    CLINK_MODBUS_CLIENT_STATUS_DEVICE_BUSY,
    /**
    @brief  모드버스 client Input error
    */
    CLINK_MODBUS_CLIENT_STATUS_INPUT_ERROR,
    /**
    @brief  모드버스 client Connection time out
    */
    CLINK_MODBUS_CLIENT_STATUS_CONNECTION_TIME_OUT
} CLINK_MODBUS_CLIENT_STATUS;

/**
@brief  모드버스 IO 상태
*/
typedef enum
{
    CLINK_MODBUS_CLIENT_IO_STATUS_BEGIN = CLINK_MODBUS_CLIENT_STATUS_BEGIN + 100,
    /**
    @brief  모드버스 IO Not connect
    */
    CLINK_MODBUS_CLIENT_IO_STATUS_NOT_CONNECT,
    /**
    @brief  모드버스 IO Normal
    */
    CLINK_MODBUS_CLIENT_IO_STATUS_NORMAL,
    /**
    @brief  모드버스 IO Device busy
    */
    CLINK_MODBUS_CLIENT_IO_STATUS_DEVICE_BUSY,
    /**
    @brief  모드버스 IO Address error
    */
    CLINK_MODBUS_CLIENT_IO_STATUS_ADDRESS_ERROR,   
    /**
    @brief  모드버스 IO Input error
    */
    CLINK_MODBUS_CLIENT_IO_STATUS_INPUT_ERROR
} CLINK_MODBUS_CLIENT_IO_STATUS;

/**
@brief  Ethernet_IP 상태
        EIP IO Status
*/
typedef enum
{
    /**
    @brief OFF : EIP disabled
    */
    CLINK_LEDSTATE_OFF = CLINK_MODBUS_CLIENT_IO_STATUS_BEGIN + 100,
    /**
    @brief FLASH_YELLOW : Not defined
    */
    CLINK_LEDSTATE_FLASH_YELLOW,
    /**
    @brief FLASH_GREEN : PLC connection is established : Program mode
    */
    CLINK_LEDSTATE_FLASH_GREEN,
    /**
    @brief FLASH_RED : Minor, recoverable fault
    */
    CLINK_LEDSTATE_FLASH_RED,
    /**
    @brief STEADY_YELLOW : EIP enabled, Standby
    */
    CLINK_LEDSTATE_STEADY_YELLOW,
    /**
    @brief STEADY_GREEN : IO connection established : Run mode
    */
    CLINK_LEDSTATE_STEADY_GREEN,
    /**
    @brief STEADY_RED : Non-recoverable fault
    */
    CLINK_LEDSTATE_STEADY_RED,
    /**
    @brief ALT_REDGREEN : Self test, the device is performing power-up testing
    */
    CLINK_LEDSTATE_ALT_REDGREEN,
} CLINK_LED_STATE;

/**
@brief  사용자 데이터 타입
*/
typedef enum
{
    CLINK_USER_DATA_NO_TYPE = CLINK_LEDSTATE_OFF + 100,
    /**
    @brief  user data int type
    */
    CLINK_USER_DATA_TYPE_INT,
    /**
    @brief  user data string type
    */
    CLINK_USER_DATA_TYPE_STRING,
    /**
    @brief  user data boolean type
    */
    CLINK_USER_DATA_TYPE_BOOL,
    /**
    @brief  user data double type
    */
    CLINK_USER_DATA_TYPE_DOUBLE,
    /**
    @brief  user data int64 type
    */
    CLINK_USER_DATA_TYPE_LONGLONG
} CLINK_USER_DATA_TYPE;

/**
@brief  UI program 상태 타입
*/
typedef enum
{
    CLINK_UI_PROGRAM_STATE_UNDEFINED = CLINK_USER_DATA_NO_TYPE + 100,
    /**
    @brief  UI program Init
    */
    CLINK_UI_PROGRAM_STATE_INIT,
    /**
    @brief  UI program Run
    */
    CLINK_UI_PROGRAM_STATE_RUN,
    /**
    @brief  UI program Pausing
    */
    CLINK_UI_PROGRAM_STATE_PAUSING,
    /**
    @brief  UI program Pause completed
    */
    CLINK_UI_PROGRAM_STATE_PAUSED,
    /**
    @brief  UI program Stopping
    */
    CLINK_UI_PROGRAM_STATE_STOPPING,
    /**
    @brief  UI program Stop completed
    */
    CLINK_UI_PROGRAM_STATE_STOPPED,
    /**
    @brief  UI program Runtime error
    */
    CLINK_UI_PROGRAM_STATE_RUNTIME_ERROR,
} CLINK_UI_PROGRAM_STATE;

/**
@brief SDO write type
*/
typedef enum
{
    CLINK_SDO_TYPE_DATA_UNDEFINED = CLINK_UI_PROGRAM_STATE_UNDEFINED + 100,
    /**
    @brief  SDO TX
    */
    CLINK_SDO_TYPE_DATA_TX,
    /**
    @brief  SDO RX
    */
    CLINK_SDO_TYPE_DATA_RX,
    /**
    @brief  SDO HOME OFFSET Setting
    */
    CLINK_SDO_TYPE_DATA_MOTOR_SET_HOME_OFFSET,
    /**
    @brief  SDO Raw TX
    */
    CLINK_SDO_TYPE_DATA_TX_DIRECT,
    /**
    @brief  SDO Raw Rx
    */
    CLINK_SDO_TYPE_DATA_RX_DIRECT
} CLINK_SDO_TYPE;

/**
@brief Control Software 및 Clink API Log에 사용되는 정보
*/
typedef enum
{
    CLINK_LOG_TYPE_UNDEFINED = CLINK_SDO_TYPE_DATA_UNDEFINED + 100,
    /**
    @brief  ERROR
    */
    CLINK_LOG_TYPE_ERROR,
    /**
    @brief  WARNING
    */
    CLINK_LOG_TYPE_WARNING,
    /**
    @brief  일반적인 정보
    */
    CLINK_LOG_TYPE_INFO,
    /**
    @brief  이벤트 발생 정보
    */
    CLINK_LOG_TYPE_EVENT,
    /**
    @brief  커맨드 관련 정보
    */
    CLINK_LOG_TYPE_CMD,
    /**
    @brief  위 정보에 추가하여 더해지는 정보, 단독 사용 X
    */
    CLINK_LOG_TYPE_ADDITIONAL
} CLINK_LOG_TYPE;

/**
@brief  IO LINK의 WP 상태 정보
*/
typedef enum
{
    CLINK_IO_LINK_WP_UNDEFINED = CLINK_LOG_TYPE_UNDEFINED + 100,
    /**
    @brief  ERROR
    */
    CLINK_IO_LINK_WP_PAIRING_SUCCESS,
    /**
    @brief  W-DEVICE HAS BEEN PAIRED
    */
    CLINK_IO_LINK_WP_PAIRING_TIMEOUT,
    /**
    @brief  W-DEVICE HASN‘T BEEN PAIRED WITHIN THE GIVEN TIMEOUT
    */
    CLINK_IO_LINK_WP_PAIRING_WRONG_SLOTTYPE,
    /**
    @brief  W-DEVICE HAS DIFFERENT SLOTTYPE THAN THAT REQUESTED
    */
    CLINK_IO_LINK_WP_INACTIVE,
    /**
    @brief  COMMUNICATION DISABLED
    */
    CLINK_IO_LINK_WP_PORTREADY,
    /**
    @brief  W-PORT CONFIGURATION SUCCESSFUL
    */
    CLINK_IO_LINK_WP_COMREADY,
    /**
    @brief  COMMUNICATION ESTABLISHED AND INSPECTION SUCCESSFUL
    */
    CLINK_IO_LINK_WP_OPERATE,
    /**
    @brief  W-PORT IS READY TO EXCHANGE PROCESS DATA
    */
    CLINK_IO_LINK_WP_COMLOST,
    /**
    @brief  COMMUNICATION FAILED, NEW SYNCHRONIZATION PROCEDURE REQUIRED
    */
    CLINK_IO_LINK_WP_REVISION_FAULT,
    /**
    @brief  INCOMPATIBLE PROTOCOL REVISION
    */
    CLINK_IO_LINK_WP_COMP_FAULT,
    /**
    @brief  INCOMPATIBLE W-DEVICE OR LEGACY-DEVICE ACCORDING TO THE INSPECTION LEVEL
    */
    CLINK_IO_LINK_WP_SERNUM_FAULT
} CLINK_IO_LINK_WP_STATUS;

/**
@brief  arc sensing 에서 사용되는 data source
*/
typedef enum
{
    /**
    @brief  EtherCAT IO analog input 0
    */
    CLINK_ARC_SENSING_SRC_ECAT_ANALOG0 = CLINK_IO_LINK_WP_UNDEFINED + 100,
    /**
    @brief  EtherCAT IO analog input 1
    */
    CLINK_ARC_SENSING_SRC_ECAT_ANALOG1,
    /**
    @brief  Robot 0 Tool IO analog input 0
    */
    CLINK_ARC_SENSING_SRC_TOOL_ANALOG0,
    /**
    @brief  Robot 0 Tool IO analog input 1
    */
    CLINK_ARC_SENSING_SRC_TOOL_ANALOG1,
    /**
    @brief  External Source
    */
    CLINK_ARC_SENSING_SRC_EXT_SRC
} CLINK_ARC_SENSING_SRC;

/**
@brief  arc sensing 에서 사용되는 보정 방향
*/
typedef enum
{
    /**
    @brief  좌우 보정
    */
    CLINK_ARC_SENSING_DIRECTION_HORIZONTAL = CLINK_ARC_SENSING_SRC_ECAT_ANALOG0 + 100,
    /**
    @brief  상하 보정
    */
    CLINK_ARC_SENSING_DIRECTION_VERTICAL
} CLINK_ARC_SENSING_DIRECTION;

/**
@brief  EIP IO DEVICE KEY 정보
*/
typedef enum
{
    /**
    @brief  UNDEFINED
    */
    CLINK_EIP_IO_DEV_UNDEFINED = CLINK_ARC_SENSING_DIRECTION_HORIZONTAL + 100,
    /**
    @brief  EIP IO DEVICE HCR_ROBOT
    */
    CLINK_EIP_IO_DEV_HCR_ROBOT,
    /**
    @brief  EIP IO DEVICE ARC_ZI_SOL350
    */
    CLINK_EIP_IO_DEV_ARC_ZI_SOL350
} CLINK_EIP_IO_DEVICE_MODEL;

/**
@brief  RS485 Serial Interface 설정을 정의한다.
*/
typedef enum
{
    CLINK_RS485_SERIAL_INTERFACE_UNDEFINED = CLINK_EIP_IO_DEV_UNDEFINED + 100,
    /**
    @brief  RS422
    */
    CLINK_RS422,
    /**
    @brief  RS422 120OHM
    */
    CLINK_RS422_120OHM,
    /**
    @brief  RS485
    */
    CLINK_RS485,
    /**
    @brief  RS485 120OHM
    */
    CLINK_RS485_120OHM
}CLINK_RS485_SERIAL_INTERFACE;

/**
@brief  RS485 Baudrate 설정을 정의한다.
*/
typedef enum
{
    CLINK_RS485_BAUDRATE_UNDEFINED = CLINK_RS485_SERIAL_INTERFACE_UNDEFINED + 100,
    /**
    @brief  BAUDRATE 1200
    */
    CLINK_RS485_BAUDRATE_1200,
    /**
    @brief  BAUDRATE 2400
    */
    CLINK_RS485_BAUDRATE_2400,
    /**
    @brief  BAUDRATE 4800
    */
    CLINK_RS485_BAUDRATE_4800,
    /**
    @brief  BAUDRATE 9600
    */
    CLINK_RS485_BAUDRATE_9600,
    /**
    @brief  BAUDRATE 19200
    */
    CLINK_RS485_BAUDRATE_19200,
    /**
    @brief  BAUDRATE 38400
    */
    CLINK_RS485_BAUDRATE_38400,
    /**
    @brief  BAUDRATE 57600
    */
    CLINK_RS485_BAUDRATE_57600,
    /**
    @brief  BAUDRATE 115200
    */
    CLINK_RS485_BAUDRATE_115200,
    /**
    @brief  BAUDRATE 230400
    */
    CLINK_RS485_BAUDRATE_230400,
    /**
    @brief  BAUDRATE 460800
    */
    CLINK_RS485_BAUDRATE_460800,
    /**
    @brief  BAUDRATE 921600
    */
    CLINK_RS485_BAUDRATE_921600,
    /**
    @brief  BAUDRATE 1000000
    */
    CLINK_RS485_BAUDRATE_1000000
}CLINK_RS485_BAUDRATE;

/**
@brief  RS485 Data frame 설정을 정의한다.
*/
typedef enum
{
    CLINK_RS485_DATA_FRAME_UNDEFINED = CLINK_RS485_BAUDRATE_UNDEFINED + 100,
    /**
    @brief  Data 7, Parity Even, Stop 1
    */
    CLINK_RS485_DATA_FRAME_7E1,
    /**
    @brief  Data 7, Parity Odd, Stop 1
    */
    CLINK_RS485_DATA_FRAME_7O1,
    /**
    @brief  Data 8, Parity None, Stop 1(Default)
    */
    CLINK_RS485_DATA_FRAME_8N1,
    /**
    @brief  Data 8, Parity Even, Stop 1
    */
    CLINK_RS485_DATA_FRAME_8E1,
    /**
    @brief  Data 8, Parity Odd, Stop 1
    */
    CLINK_RS485_DATA_FRAME_8O1,
    /**
    @brief  Data 7, Parity Even, Stop 2
    */
    CLINK_RS485_DATA_FRAME_7E2,
    /**
    @brief  Data 7, Parity Odd, Stop 2
    */
    CLINK_RS485_DATA_FRAME_7O2,
    /**
    @brief  Data 8, Parity None, Stop 2
    */
    CLINK_RS485_DATA_FRAME_8N2,
    /**
    @brief  Data 8, Parity Even, Stop 2
    */
    CLINK_RS485_DATA_FRAME_8E2,
    /**
    @brief  Data 8, Parity Odd, Stop 2
    */
    CLINK_RS485_DATA_FRAME_8O2

}CLINK_RS485_DATA_FRAME;

/**
@brief  RS485 Protocol 설정을 정의한다.
*/
typedef enum
{
    CLINK_RS485_PROTOCOL_UNDEFINED = CLINK_RS485_DATA_FRAME_UNDEFINED + 100,
    /**
    @brief  Standard (Beckhoff compatible)
    */
    CLINK_RS485_PROTOCOL_STANDARD,
    /**
    @brief  Modbus RTU
    */
    CLINK_RS485_PROTOCOL_MODBUS_RTU,
    /**
    @brief  Modbus ASCII
    */
    CLINK_RS485_PROTOCOL_MODBUS_ASCII
}CLINK_RS485_PROTOCOL;

/**
@brief  RS485 Control Command 설정을 정의한다.
*/
typedef enum
{
    CLINK_RS485_CMD_UNDEFINED = CLINK_RS485_PROTOCOL_UNDEFINED + 100,
    /**
    @brief  TX REQUEST
    */
    CLINK_RS485_CMD_TX_REQUEST,
    /**
    @brief  RX ACCEPTED
    */
    CLINK_RS485_CMD_RX_ACCEPTED,
    /**
    @brief  RX INIT REQUEST
    */
    CLINK_RS485_CMD_RX_INIT_REQUEST,
    /**
    @brief  TX CONTINUOUS ON
    */
    CLINK_RS485_CMD_TX_CONTINUOUS_ON,
    /**
    @brief  TX CONTINUOUS OFF
    */
    CLINK_RS485_CMD_TX_CONTINUOUS_OFF,
    /**
    @brief  TX SCAN RATE REQUEST ON
    */
    CLINK_RS485_CMD_TX_SCAN_RATE_REQUEST_ON,
    /**
    @brief  TX SCAN RATE REQUEST OFF
    */
    CLINK_RS485_CMD_TX_SCAN_RATE_REQUEST_OFF,
    /**
    @brief  RX PAUSE ON
    */
    CLINK_RS485_CMD_RX_PAUSE_ON,
    /**
    @brief  RX PAUSE OFF
    */
    CLINK_RS485_CMD_RX_PAUSE_OFF
}CLINK_RS485_CONTROL_CMD;

/**
@brief  로봇 FT Sensor 장착 여부를 나타낸다.
*/
typedef enum
{
    /**
    @brief  장착되지 않음
    */
    CLINK_ROBOT_FT_SENSOR_TYPE_EMPTY = CLINK_RS485_CMD_UNDEFINED + 100,
    /**
    @brief  ATI AXIA80 모델
    */
    CLINK_ROBOT_FT_SENSOR_TYPE_AXIA80,
}CLINK_ROBOT_FT_SENSOR_TYPE;

/**
@brief  FSOE Digital IO INPUT에서 가져올 데이터 설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  FSOE 데이터 설정하지 않음
    */
    CLINK_FSOE_INPUT_EMPTY = CLINK_ROBOT_FT_SENSOR_TYPE_EMPTY + 100,
    /**
    @brief  RESET 데이터 설정
    */
    CLINK_FSOE_RESET_INPUT,
    /**
    @brief  MC 데이터 설정
    */
    CLINK_FSOE_MC_INPUT,
    /**
    @brief  STO1 데이터 설정
    */
    CLINK_FSOE_STO1_INPUT,
    /**
    @brief  STO2 데이터 설정
    */
    CLINK_FSOE_STO2_INPUT,
    /**
    @brief  QSTOP 데이터 설정
    */
    CLINK_FSOE_QSTOP_INPUT,
    /**
    @brief  FRONT_OSSD_A 데이터 설정
    */
    CLINK_FSOE_FRONT_OSSD_A_INPUT,
    /**
    @brief  FRONT_OSSD_B 데이터 설정
    */
    CLINK_FSOE_FRONT_OSSD_B_INPUT,
    /**
    @brief  REAR_OSSD_A 데이터 설정
    */
    CLINK_FSOE_REAR_OSSD_A_INPUT,
    /**
    @brief  REAR_OSSD_B 데이터 설정
    */
    CLINK_FSOE_REAR_OSSD_B_INPUT,
    /**
    @brief  FSOE INPUT MAX 값
    */
    CLINK_FSOE_INPUT_MAX
}CLINK_FSOE_IO_INPUT_DATA;

/**
@brief  FSOE Digital IO OUTPUT에 데이터를 읽거나 설정할 때 사용한다.
*/
typedef enum
{
    /**
    @brief  FSOE 데이터 설정하지 않음
    */
    CLINK_FSOE_OUTPUT_EMPTY = CLINK_FSOE_INPUT_EMPTY + 100,
    /**
    @brief  MC3 POWER 데이터 설정
    */
    CLINK_FSOE_MC3_POWER_OUTPUT,    
    /**
    @brief  FSOE OUTPUT MAX 값
    */
    CLINK_FSOE_OUTPUT_MAX
}CLINK_FSOE_IO_OUTPUT_DATA;

/**
@brief  system 내부 사용
*/
typedef enum
{
    CLINK_SYSTEM_DATA_VALUE = CLINK_FSOE_OUTPUT_EMPTY + 100
} CLINK_SYSTEM_DATA;

/** @} */
/** @} */

#ifdef __cplusplus
}
#endif


#endif // __CLINK_CONSTANT_H__

