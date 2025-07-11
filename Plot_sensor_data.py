import pandas as pd
import matplotlib.pyplot as plt


# Specify the column names
column_names = ['Timestamp','ReadNumber','Source','SensorName','Parameter','Value']

# Load the CSV file with the specified column names
df = pd.read_csv('sensor_data.csv', delimiter=",",  encoding=('utf-8'), names=column_names, skiprows=20)

# Split the 'SensorName', 'Parameter'(often a Gas), and 'Value' columns at the colon and take the second part
#df['Sensor'] = df['SensorName'].str.split(':', expand=True)[1]
#df['Gas'] = df['Parameter'].str.split(':', expand=True)[1]
#df['Value'] = df['Value'].str.split(':', expand=True)[1]

# Convert 'Value' to numeric if it's supposed to be a number
df['Value'] = pd.to_numeric(df['Value'], errors='coerce')

# # Convert the 'Timestamp' column to datetime
# df['Timestamp'] = pd.to_datetime(df['Timestamp'])

# # Set the index to 'Timestamp'
# df.set_index('Timestamp', inplace=True)

# Assuming 'df' is your DataFrame
df['Value'] = df['Value'].fillna(0)  # Fill missing values with 0


# Define the gases you want to plot
gases = ['CO2','TVOC','CH4', 'CO']

# Initialize a figure with subplots
fig, axs = plt.subplots(len(gases), 1, figsize=(10, 20))

# Loop through each gas and plot the data for Arduino0 and Arduino1
for i, gas in enumerate(gases):
    # Filter the data for the current gas
    arduino0_data = df[(df['Source'] == 'Arduino0') & (df['Parameter'] == gas)]['Value']
    arduino1_data = df[(df['Source'] == 'Arduino1') & (df['Parameter'] == gas)]['Value']

    
    # Plot the data
    axs[i].plot(arduino0_data, label='Arduino0', color='blue')
    axs[i].plot(arduino1_data, label='Arduino1', color='red')
    
    # Set the title and labels for the subplot
    axs[i].set_title(f"{gas} Levels")
    axs[i].set_xlabel('Read')
    axs[i].set_ylabel('Value')
    
    # Add a legend
    axs[i].legend()

# Adjust the layout for better readability
plt.tight_layout()

# Show the plot
plt.show()
