# ESP32 Gas Monitoring System

An IoT-based gas monitoring project using ESP32, an HTML dashboard, and Supabase integration. The project is designed to collect gas sensor readings and present monitoring information through a web interface.

## Features

- ESP32-based gas monitoring
- Web dashboard built with HTML, CSS, and JavaScript
- Supabase integration for cloud data storage
- Dashboard charts using Chart.js
- OLED display support, depending on the connected hardware
- Optional Telegram notifications, if configured

## Technologies Used

- **Hardware:** ESP32 and compatible gas sensor
- **Firmware:** Arduino IDE and C++
- **Frontend:** HTML, CSS, JavaScript
- **Dashboard charts:** Chart.js
- **Cloud backend:** Supabase

## Project Structure

```text
esp32-gas-monitor/
├── esp32_gas_monitor_v5_supabase.ino
└── dashboard/
    └── index.html
```

## Requirements

- ESP32 development board
- Compatible gas sensor
- USB cable and computer
- Arduino IDE
- Wi-Fi connection
- Supabase project, if cloud functionality is required

Additional hardware such as an OLED display is required only if used in your setup.

## Setup Instructions

### 1. Configure the ESP32

1. Install Arduino IDE.
2. Open `esp32_gas_monitor_v5_supabase.ino`.
3. Install the required libraries and select the correct ESP32 board.
4. Configure Wi-Fi and cloud settings using your local credentials.
5. Connect the sensor and any optional display according to your circuit.

### 2. Configure Supabase

1. Create or select a Supabase project.
2. Configure the required database tables and access policies.
3. Use only the appropriate publishable key in client-side code, with suitable Row Level Security policies.
4. Never commit secret or service-role keys, Wi-Fi passwords, or other credentials to GitHub.

### 3. Open the Web Dashboard

Open `dashboard/index.html` in a browser to view the dashboard interface.

Cloud-connected features require the correct configuration and a running ESP32. Opening the HTML file locally does not automatically deploy it as a public website.

## Security

Keep all private credentials out of the repository. Use placeholders in shared code and configure real credentials locally. If a credential has been exposed, revoke or rotate it.

## Future Improvements

- Live dashboard deployment
- Improved sensor visualization and historical charts
- Alerts for configurable gas thresholds
- Better documentation and hardware setup diagrams

## Author

**Resham Nandini**

## License

No license has been added yet. Contact the repository owner before reusing or distributing this project's code.
