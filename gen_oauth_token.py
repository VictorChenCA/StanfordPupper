import requests
import dotenv
import os 

dotenv.load_dotenv()

# Twitch OAuth token endpoint
url = 'https://id.twitch.tv/oauth2/token'

# Request payload
payload = {
    'client_id': os.getenv('CLIENT_ID'),
    'client_secret': os.getenv('CLIENT_SECRET'),
    'grant_type': 'client_credentials'
}

# Send POST request
response = requests.post(url, data=payload)

# Parse and print the token
if response.status_code == 200:
    data = response.json()
    print("✅ Access Token:", data['access_token'])
    print("⏳ Expires In:", data['expires_in'], "seconds")
    print("🔐 Token Type:", data['token_type'])
else:
    print("❌ Error:", response.status_code, response.text)