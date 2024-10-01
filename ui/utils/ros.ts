// @ts-nocheck

/**
 * @file ros.ts
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief ROS connection
 *
 * @copyright Copyright (c) 2024
 *
 */
"use client";

import { useState, useEffect } from 'react';
import ROSLIB from 'roslib';
import { useToast } from "@/hooks/use-toast"
import { useStatusStore } from '@/store/statusStore';

export function useROSConnection() {
  const [isConnected, setIsConnected] = useState(false);
  // const { connectedToROS, setConnectedToROS} = useStatusStore();
  const { toast } = useToast();

  useEffect(() => {
    let ros;
    let reconnectInterval;

    const connect = () => {
      ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090' // Replace with your ROS server URL
      });

      ros.on('connection', () => {
        console.log('Connected to ROS server');
        // setConnectedToROS(true);
        setIsConnected(true);
        toast({
          title: "Connected 🎉",
          description: "Successfully connected to ROS server",
        });
        if (reconnectInterval) clearInterval(reconnectInterval);
      });

      ros.on('error', (error) => {
        console.error('Error connecting to ROS server:', error);
        // setConnectedToROS(false);
        setIsConnected(false);
      });

      ros.on('close', () => {
        console.log('Connection to ROS server closed');
        // setConnectedToROS(false);
        setIsConnected(false);
        toast({
          title: "Disconnected",
          description: "Connection to ROS server lost. Retrying...",
          variant:"destructive",
          duration: 5000,
        });
      });
    };

    connect();

    reconnectInterval = setInterval(() => {
      if (!isConnected) {
        connect();
      }
    }, 10000); // Retry every 5 seconds

    return () => {
      if (ros) ros.close();
      if (reconnectInterval) clearInterval(reconnectInterval);
    };
  }, []);

  return { isConnected };
}
