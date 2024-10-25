import numpy as np
import scipy.stats as stats

def read_sample(file_path):
    # Open the file and read the data
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    # Extract the specified 10 data points
    # Read the last part of the data
    data = []
    for line in lines[-10:]:
        data.append(float(line.strip().split(',')[0]))
    
    return data

# File paths
file_path_A = "./large_timelimit/1000_4_timelimit=32400_subproblem=1.txt"
file_path_B = "./large_timelimit/1000_4_timelimit=32400_subproblem=10.txt"

# Read sample data
sample_A = read_sample(file_path_A)
sample_B = read_sample(file_path_B)

# Calculate the p-value of the Wilcoxon rank-sum test
stat, p_value = stats.ranksums(sample_A, sample_B)

# Round the p-value to 4 decimal places
p_value_rounded = round(p_value, 4)

# Calculate mean and standard deviation
mean_A = np.mean(sample_A)
std_A = np.std(sample_A, ddof=1)  # ddof=1 for sample standard deviation
mean_B = np.mean(sample_B)
std_B = np.std(sample_B, ddof=1)  # ddof=1 for sample standard deviation

print("Sample A:", sample_A)
print("Sample B:", sample_B)
print("Statistic:", stat)
print(f"p-value: {p_value_rounded:.4f}")
print(f"Sample A Mean: {mean_A:.2f}, Standard Deviation: {std_A:.2f}")
print(f"Sample B Mean: {mean_B:.2f}, Standard Deviation: {std_B:.2f}")
