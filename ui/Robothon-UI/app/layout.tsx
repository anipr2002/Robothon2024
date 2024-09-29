// @ts-nocheck
"use client";
// import type { Metadata } from "next";
import React, { useState, useEffect } from "react";
import localFont from "next/font/local";
import { roger, animal, handwriting } from "./fonts/font";

import "./globals.css";
import { ThemeProvider } from "@/components/theme-provider";

import { useROSConnection } from "@/utils/ros";
import { useRobotStore } from "@/store/robotStore";

import { Toaster } from "@/components/ui/toaster";
import { Spinner } from "@/components/ui/spinner";
import { Button } from "@/components/ui/button";
import { PlayIcon, PauseIcon } from "lucide-react";

import { set_speed_slider_service } from "@/utils/services";
import tcp_listener from "@/utils/topics";

const geistSans = localFont({
  src: "./fonts/GeistVF.woff",
  variable: "--font-geist-sans",
  weight: "100 900",
});
const geistMono = localFont({
  src: "./fonts/GeistMonoVF.woff",
  variable: "--font-geist-mono",
  weight: "100 900",
});

// export const metadata: Metadata = {
//   title: "Robothon UI",
//   description: "Dashboard for Robothon 2024",
// };

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  // const { connectedToROS } = useStatusStore();
  const { isConnected } = useROSConnection();

  console.log("isConnected", isConnected);

  const [isPlaying, setIsPlaying] = useState(false);
  const togglePlayPause = () => {
    const newSpeed = isPlaying ? 0 : 0.2;
    setIsPlaying(!isPlaying);

    set_speed_slider_service({ slider_args: newSpeed });
  };

  const { setRobotPose } = useRobotStore();

  useEffect(() => {
    tcp_listener.subscribe((msg) => {
      setRobotPose(msg.transform);
    });
  }, []);
  return (
    <html lang="en">
      <body
        className={`${geistSans.variable} ${geistMono.variable} ${roger.variable} ${animal.variable} ${handwriting.variable} antialiased`}
      >
        <ThemeProvider
          attribute="class"
          defaultTheme="light"
          // enableSystem
          // disableTransitionOnChange
        >
          {!isConnected && (
            <div className="fixed top-0 left-0 right-0 bg-yellow-300 text-[#f0721b] p-2 text-center">
              <div className="flex items-center justify-center gap-3">
                Connecting to ROS server
                <Spinner className="text-white" />
              </div>
            </div>
          )}

          <div className="absolute top-4 right-4">
            <Button onClick={togglePlayPause} className="shadow-lg">
              {isPlaying ? (
                <PauseIcon className="h-4 w-4" />
              ) : (
                <PlayIcon className="h-4 w-4" />
              )}
            </Button>
          </div>

          {children}
        </ThemeProvider>
        <Toaster />
      </body>
    </html>
  );
}
