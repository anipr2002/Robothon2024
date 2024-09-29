/**
 * @file page.tsx
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief Services page component
 *
 * @copyright Copyright (c) 2024
 *
 */
"use client";
import React, { useEffect, useState } from "react";
import SpeedSlider from "./_components/SpeedSlider";
import BoardDetection from "./_components/BoardDetection";
import GripperMove from "./_components/GripperMove";
import Freedrive from "./_components/Freedrive";

// import { tcp_pose_listener } from "@/utils/topics";
const page = () => {
  // useEffect(() => {
  //   tcp_pose_listener();
  // }, []);

  return (
    <>
      <div className="flex flex-col w-full h-screen p-8">
        <div className="text-6xl font-roger font-semibold">Services</div>
        <div className="w-full h-full flex gap-7 mt-5">
          <SpeedSlider />
          <BoardDetection />
          <GripperMove />
          <Freedrive />
        </div>
      </div>
    </>
  );
};

export default page;
