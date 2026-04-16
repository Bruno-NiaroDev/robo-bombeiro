const express = require('express');
const http = require('http');
const url = require('url');
const WebSocket = require('ws');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

const channelMap = {};

wss.on('connection', (ws, req) => {

    const { pathname, query } = url.parse(req.url, true);
    const channelCode = query.channelCode.toString();
    const hook = query.hook === 'true';

    // const { channelCode, hook } = req.query;
    console.log(`Client connected with channelCode: ${channelCode}`);

    ws.on('message', (data) => {
        try {
            const message = JSON.parse(data);
            if (channelMap[channelCode]) {
                channelMap[channelCode].forEach((client) => {
                    if (client !== ws && client.readyState === WebSocket.OPEN && !client.isWebhook) {
                        console.log("send to " + channelCode);
                        client.send(JSON.stringify(message));
                    }
                });
            }
        } catch (error) {
            console.error('Error parsing message:', error);
        }
    });

    ws.on('close', () => {
        console.log(`Client disconnected with channelCode: ${channelCode}`);
        if (channelMap[channelCode]) {
            channelMap[channelCode] = channelMap[channelCode].filter((client) => client !== ws);
            if (channelMap[channelCode].length === 0) {
                delete channelMap[channelCode];
            }
        }
    });

    if (!channelMap[channelCode]) {
        channelMap[channelCode] = [];
    }

    ws.isWebhook = hook === 'true';
    channelMap[channelCode].push(ws);
});

const PORT = 55619;
server.listen(PORT, () => {
    console.log(`Server listening on port ${PORT}`);
});
