/**
 * @file page.tsx
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief Task page component
 *
 * @copyright Copyright (c) 2024
 *
 */
import React from "react";
import TaskFlow from "../_components/TaskFlow";
import RobotStatus from "../_components/RobotStatus";
const page = () => {
  return (
    <>
      <TaskFlow />
      <RobotStatus />
    </>
  );
};

export default page;
