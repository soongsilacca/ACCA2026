from DB import DB
import numpy as np
from scipy.interpolate import CubicSpline

a = DB("B1_1.db")
row = a.read_from_idx_to_path(100)


def interpolate_path():
        x = np.array(self.path_x)
        y = np.array(self.path_y)
        dx_ = np.diff(x)
        dy_ = np.diff(y)
        ds = np.sqrt(dx_**2 + dy_**2)
        s = np.concatenate([[0], np.cumsum(ds)])
        try:
            cs_x = CubicSpline(s, x, bc_type="natural")
            cs_y = CubicSpline(s, y, bc_type="natural")
            
            self.narrow = int(s[-1] / self.ds)

            s_new = np.linspace(s[0], s[-1], self.narrow)
            
            self.get_logger().info(f"path_idx_num: {self.narrow}.")
            x_new = cs_x(s_new)
            y_new = cs_y(s_new)

            dx_new = cs_x(s_new, 1)
            dy_new = cs_y(s_new, 1)

            yaw_new = [m.atan2(dy, dx) for dy, dx in zip(dy_new, dx_new)]

            self.path = list(zip(x_new.tolist(), y_new.tolist(), yaw_new))
            
            # plt.figure()
            # plt.plot(x, y, 'o', label='data points')
            # plt.plot(x_new, y_new, '-', label='cubic spline')
            # plt.legend(loc='best')
            # plt.title('Cubic Spline')
            # plt.show()
            
            self.write_db()
        except Exception as e:
            self.get_logger().error(f"An error occurred during spline interpolation: {e}")
            
def write_db(self):
    self.db.write_db_Path(self.path)
