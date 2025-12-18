import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class ParamConfigurer:
    def __init__(self, node):
        self.node = node
        self.cli_cropbox = self.node.create_client(
            SetParameters, "/bs_cropbox_filter/set_parameters"
        )

        self.cli_gps = self.node.create_client(
            SetParameters, "/gps_jamming_filter/set_parameters"
        )

        if not self.cli_cropbox.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("파라미터 서비스가 없습니다")

        if not self.cli_gps.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error("파라미터 서비스가 없습니다")

    def set_detection_area(
        self, area: list[float], retries: int = 3
    ) -> SetParameters.Response:
        """
        area: [min_x, max_x, min_y, max_y]
        retries: 시도 횟수 (기본 3회)
        """
        last_exc = None

        req = SetParameters.Request()
        p = Parameter()
        p.name = "detection_area"
        p.value = ParameterValue(
            type=ParameterType.PARAMETER_DOUBLE_ARRAY, double_array_value=list(area)
        )
        req.parameters = [p]

        for attempt in range(retries):
            try:
                fut = self.cli_cropbox.call_async(req)
                rclpy.spin_until_future_complete(self.node, fut, timeout_sec=0.05)

                ## 서비스는 성공했으나, 파라미터가 거부
                resp = fut.result()
                success = resp.results[0].successful
                if not success:
                    reason = resp.results[0].reason or "알 수 없는 이유"
                    raise RuntimeError(f"파라미터 거부됨: {reason}")

            except Exception as e:
                last_exc = e
                self.node.get_logger().warn(
                    f"set_detection_area 시도 {attempt} 실패: {e}"
                )
                # 기회가 남은 경우 재시도
                continue

            return resp

        # 모든 시도가 실패했을 때
        raise RuntimeError(
            f"set_detection_area 실패: {retries}회 시도 후에도 반응 없음"
        ) from last_exc

    def set_gps_jamming(self, enable: bool, retries: int = 3) -> SetParameters.Response:
        """
        enable: GPS 재밍 모드 활성(True)/비활성(False)
        retries: 시도 횟수 (기본 3회)
        """
        last_exc = None

        # 1) 요청 메시지 미리 구성
        req = SetParameters.Request()
        p = Parameter()
        p.name = "gps_jamming_mode"
        p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enable)
        req.parameters = [p]

        # 2) 재시도 루프
        for attempt in range(1, retries + 1):
            try:
                fut = self.cli_gps.call_async(req)
                rclpy.spin_until_future_complete(self.node, fut, timeout_sec=0.05)

                # 3) 서비스는 응답했지만, 노드 내부에서 거부된 경우
                resp = fut.result()
                success = resp.results[0].successful
                if not success:
                    reason = resp.results[0].reason or "알 수 없는 이유"
                    raise RuntimeError(f"파라미터 거부됨: {reason}")

                # 성공 시 결과 반환
                return resp

            except Exception as e:
                last_exc = e
                self.node.get_logger().warn(f"set_gps_jamming 시도 {attempt} 실패: {e}")
                # 남은 기회가 있으면 곧바로 다음 루프 진입
                continue

        # 4) 모든 시도 실패 시 예외 발생
        raise RuntimeError(
            f"set_gps_jamming 실패: {retries}회 시도 후에도 반응 없음"
        ) from last_exc


def main():
    rclpy.init()
    node = Node("bum")
    param = ParamConfigurer(node)
    res = param.set_detection_area([0.0, 3.0, -10.0, 0.0])
    node.get_logger().info(f"Detection Area 결과: {res.results}")
    res_gps = param.set_gps_jamming(True)
    node.get_logger().info(f"GPS 재밍 결과: {res_gps.results}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
