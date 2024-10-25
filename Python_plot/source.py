import psycopg2
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns


conn = psycopg2.connect(
    dbname="wildfires",
    user="dev",
    password="dev",
    host="localhost",
    port="5432"
)

query = """
SELECT id, device_id, co2_ppm as CO2, co_ppm as CO, pm25, temperature, humidity, timestamp 
FROM sensor_data 
WHERE timestamp BETWEEN '2024-10-24 13:23' AND '2024-10-24 14:23'"""
df = pd.read_sql(query, conn)

conn.close()

mean_value = df['co'].mean()

plt.figure(figsize=(10, 6))
sns.lineplot(data=df, x='timestamp', y='co')

plt.text(x=df['timestamp'].max(), y=df['co'].max(), 
         s=f'Mean Sensor Rs/Ro: {mean_value:.2f}', 
         fontsize=12, color='red', 
         horizontalalignment='right', verticalalignment='top')

plt.title('CO Sensor Resistance')
plt.xlabel('Time')
plt.ylabel('Rs/Ro')
plt.legend()
plt.show()