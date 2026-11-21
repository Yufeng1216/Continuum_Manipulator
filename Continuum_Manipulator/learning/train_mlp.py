"""
Train an MLP that maps the end-effector pose (x, y, z, roll, pitch, yaw) to actuator
commands (4 servo positions, 4 stepper pulses, 2 chamber pressures).

This is the pure neural-network version described in the paper. The deployed hybrid
model in ../deploy/models predicts PCC parameters (L, alpha, theta) instead of stepper
pulses; see the main README.

Usage:  python train_mlp.py      (reads ../data/Training_Data.csv)
"""
import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.losses import MeanSquaredError
from sklearn.metrics import r2_score, mean_squared_error
import joblib

# 定义四元数转欧拉角的函数
def quaternion_to_euler(x, y, z, w):
    """
    将四元数转换为欧拉角 (roll, pitch, yaw)
    """
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = np.arcsin(sinp) if abs(sinp) <= 1 else np.sign(sinp) * np.pi / 2

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# 1. 数据加载与预处理
import os
DATA_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data', 'Training_Data.csv')
data = pd.read_csv(DATA_PATH)

# 查看数据基本信息
print(data.head())
print(data.info())

# 处理缺失值
if data.isnull().sum().any():
    data = data.dropna()

# 四元数转换为欧拉角
data[['roll', 'pitch', 'yaw']] = data.apply(
    lambda row: pd.Series(quaternion_to_euler(row['AR_Orientation_X'], 
                                              row['AR_Orientation_Y'], 
                                              row['AR_Orientation_Z'], 
                                              row['AR_Orientation_W'])), axis=1
)

# 输入和输出特征
input_features = ['AR_Pose_X', 'AR_Pose_Y', 'AR_Pose_Z', 'roll', 'pitch', 'yaw']
output_features = ['Servo1_Pos', 'Servo2_Pos', 'Servo3_Pos', 'Servo4_Pos',
                   'Step_Motor1', 'Step_Motor2', 'Step_Motor3', 'Step_Motor4',
                   'Pressure_1', 'Pressure_2']

X = data[input_features].values
y = data[output_features].values

# 数据集划分
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# 数据归一化
scaler_X = StandardScaler()
scaler_y = StandardScaler()

X_train = scaler_X.fit_transform(X_train)
X_test = scaler_X.transform(X_test)

y_train = scaler_y.fit_transform(y_train)
y_test = scaler_y.transform(y_test)

# 2. 构建模型
model = Sequential([
    Dense(256, activation='relu', input_shape=(X_train.shape[1],)),
    Dense(256, activation='relu'),
    Dense(y_train.shape[1])
])

# 编译模型
model.compile(optimizer='adam', loss=MeanSquaredError(), metrics=['mae'])
model.summary()

# 3. 训练模型
history = model.fit(
    X_train, y_train,
    epochs=100,
    batch_size=32,
    validation_split=0.2,
    callbacks=[
        tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=10, restore_best_weights=True)
    ]
)

# 保存整个模型到本地文件
model.save('my_model_euler.h5')  # 保存为 HDF5 格式文件
print("模型已保存为 my_model_euler.h5")

# 保存缩放器
joblib.dump(scaler_X, 'scaler_X_euler.pkl')
joblib.dump(scaler_y, 'scaler_y_euler.pkl')
print("缩放器已保存！")

# 4. 评估模型
test_loss, test_mae = model.evaluate(X_test, y_test)
print(f'测试集均方误差（MSE）：{test_loss}')
print(f'测试集平均绝对误差（MAE）：{test_mae}')

# 进行预测
y_pred_scaled = model.predict(X_test)

# 反归一化预测结果和真实值
y_pred = scaler_y.inverse_transform(y_pred_scaled)
y_true = scaler_y.inverse_transform(y_test)

# 计算 R2 和 MSE
r2 = r2_score(y_true, y_pred, multioutput='uniform_average')
mse = mean_squared_error(y_true, y_pred, multioutput='uniform_average')

print(f'测试集决定系数（R2）：{r2}')
print(f'测试集均方误差（MSE）：{mse}')

# 5. 预测新数据
new_data = np.array([[0.9258422265466894,-0.051106602,-0.09875904,1.6385985502788234,0.1614066609855818,-1.548460871]])  # 示例新数据，roll, pitch, yaw 替换为欧拉角

# 预处理新数据
new_data_scaled = scaler_X.transform(new_data)

# 预测
predictions_scaled = model.predict(new_data_scaled)

# 反归一化
predictions = scaler_y.inverse_transform(predictions_scaled)

print('预测结果（电机编码值和气压值）：')
print(predictions)
