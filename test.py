import streamlit as st
import time
import numpy as np
import pandas as pd
import serial

ser = serial.Serial(port='/dev/cu.usbmodem21201', baudrate=9600)

data = []

time.sleep(1)

placeholder = st.empty()

ser.readline()

while True:

    with placeholder:
        st.title('Arc Cube Real-Time Telemetry')

        chart_data = np.array(data)
        cols = st.columns(3)
        if len(data) != 0:
            with cols[0]:
                st.text('Acceleration (x)')
                st.line_chart(data=chart_data[:, 0])
                st.text('Gyroscope (x)')
                st.line_chart(data=chart_data[:, 3])
                st.text('Temperature')
                st.line_chart(data=chart_data[:, 6])
            with cols[1]:
                st.text('Acceleration (y)')
                st.line_chart(data=chart_data[:, 1])
                st.text('Gyroscope (y)')
                st.line_chart(data=chart_data[:, 4])
                st.text('Ultrasonic Distance')
                st.line_chart(data=chart_data[:, 8])
            with cols[2]:
                st.text('Acceleration (z)')
                st.line_chart(data=chart_data[:, 2])
                st.text('Gyroscope (z)')
                st.line_chart(data=chart_data[:, 5])
                st.text('Photoresistance')
                st.line_chart(data=chart_data[:, 9])

    try:
        line = ser.readline().decode().split(',')
        current_input = []
        for i in range(10):
            current_input.append(float(line[i]))
        data.append(current_input)

        data = data[-min(len(data),100):]
    except:
        continue

