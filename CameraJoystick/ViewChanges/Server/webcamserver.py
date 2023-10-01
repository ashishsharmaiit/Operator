const WebSocket = require('ws');

const wss = new WebSocket.Server({ port: 8083 });

const clients = new Set();

wss.on('connection', ws => {
    console.log('Client connected');
  
    // Add the client to the set of connected clients
    clients.add(ws);
  
    // Event listener for messages from clients
    ws.on('message', message => {
      console.log('Message from client:', message);
  
      // Parse the message as JSON
      const parsedMessage = JSON.parse(message);
  
      // Handle different message types
      if (parsedMessage.type === 'offer') {
        const offer = parsedMessage.sdp;
        console.log('Received SDP offer:', offer);
  
        // Send the acceptance message back to the client
        relay(JSON.stringify({type:'offer', sdp: offer, frame1: parsedMessage.frame1, frame2: parsedMessage.frame2, video_size1: parsedMessage.video_size1, video_size2: parsedMessage.video_size2, codec1: parsedMessage.codec1, codec2: parsedMessage.codec2, audio_codec: parsedMessage.audio_codec}), ws);
        console.log('Offer I am sending is:', JSON.stringify({type:'offer', sdp: offer}));
  
      } else if (parsedMessage.type === 'answer') {
        const answer = parsedMessage.sdp;
        console.log('Received SDP answer:', answer);
  
        // Send the acceptance message back to the client
        relay(message, ws);
        console.log('Answer I am sending is:', JSON.stringify({type:'answer',sdp:answer}));
  
      } else if (parsedMessage.type === 'candidate') {
        const candidate = parsedMessage.candidate;
        console.log('Received ICE Candidate:', candidate);
  
        // Send the acceptance message back to the client
        relay(JSON.stringify(candidate), ws);
      }
    });
  
    // Event listener for client disconnections
    ws.on('close', () => {
      console.log('Client disconnected');
  
      // Remove the client from the set of connected clients
      clients.delete(ws);
    });
  });
  
  // Function to broadcast a message to all clients except the sender
  function reply(message, sender) {
    clients.forEach(client => {
      if (client === sender && client.readyState === WebSocket.OPEN) {
        client.send(message);
      }
    });
  }
  
  function relay(message, sender) {
    clients.forEach(client => {
      if (client !== sender && client.readyState === WebSocket.OPEN) {
        client.send(message);
      }
    });
  }
  