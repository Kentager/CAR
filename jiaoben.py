import pandas as pd

# 读取CSV文件
df = pd.read_csv('data.csv')

# 计算各项平均值
averages = df.mean(numeric_only=True)

# 打印结果
print("各项平均值：")
print(averages)

# 更详细的格式化输出
print("\n详细平均值：")
print(f"x1的平均值: {averages['x1']:.4f}")
print(f"y1的平均值: {averages['y1']:.4f}")
print(f"z1的平均值: {averages['z1']:.4f}")
print(f"x2的平均值: {averages['x2']:.4f}")
print(f"y2的平均值: {averages['y2']:.4f}")
print(f"z2的平均值: {averages['z2']:.4f}")
