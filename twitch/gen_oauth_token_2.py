import os
import webbrowser
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs
from dotenv import load_dotenv, set_key

load_dotenv()

# Get these from https://dev.twitch.tv/console/apps
CLIENT_ID = os.getenv('TWITCH_CLIENT_ID')
CLIENT_SECRET = os.getenv('TWITCH_CLIENT_SECRET')
REDIRECT_URI = 'http://localhost:3000'

# Scopes for chat bot
SCOPES = [
    'chat:read',
    'chat:edit',
]

class OAuthHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        query = urlparse(self.path).query
        params = parse_qs(query)
        
        if 'code' in params:
            auth_code = params['code'][0]
            
            # Exchange code for token
            import requests
            
            token_url = 'https://id.twitch.tv/oauth2/token'
            data = {
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
                'code': auth_code,
                'grant_type': 'authorization_code',
                'redirect_uri': REDIRECT_URI
            }
            
            response = requests.post(token_url, data=data)
            
            if response.status_code == 200:
                token_data = response.json()
                access_token = token_data['access_token']
                
                # Save to .env file
                env_file = '.env'
                set_key(env_file, 'TWITCH_OAUTH_TOKEN', access_token)
                
                print(f"\n✅ Token obtained and saved to .env!")
                print(f"   Token: {access_token[:20]}...")
                
                self.send_response(200)
                self.send_header('Content-type', 'text/html')
                self.end_headers()
                self.wfile.write(b'<html><body><h1>Success!</h1><p>Token saved to .env file. You can close this window.</p></body></html>')
                
                # Shutdown server
                import threading
                threading.Thread(target=self.server.shutdown).start()
            else:
                print(f"❌ Error getting token: {response.text}")
                self.send_response(500)
                self.end_headers()
        else:
            self.send_response(400)
            self.end_headers()
    
    def log_message(self, format, *args):
        pass  # Suppress log messages

def generate_token():
    if not CLIENT_ID or not CLIENT_SECRET:
        print("❌ Please set TWITCH_CLIENT_ID and TWITCH_CLIENT_SECRET in your .env file")
        print("\n📝 Get these from: https://dev.twitch.tv/console/apps")
        return
    
    # Build authorization URL
    scope_string = '+'.join(SCOPES)
    auth_url = (
        f"https://id.twitch.tv/oauth2/authorize"
        f"?client_id={CLIENT_ID}"
        f"&redirect_uri={REDIRECT_URI}"
        f"&response_type=code"
        f"&scope={scope_string}"
    )
    
    print("🔐 Opening browser for authentication...")
    print(f"\nIf the browser doesn't open, go to:\n{auth_url}\n")
    
    # Start local server
    server = HTTPServer(('localhost', 3000), OAuthHandler)
    
    # Open browser
    webbrowser.open(auth_url)
    
    print("⏳ Waiting for OAuth callback...")
    server.serve_forever()

if __name__ == "__main__":
    print("🚀 Twitch OAuth Token Generator\n")
    generate_token()