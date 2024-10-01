/**
 * @file robotStore.ts
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief Robot Store
 *
 * @copyright Copyright (c) 2024
 *
 */
import { create } from 'zustand';


type RobotPose = {
    translation:{
        x: number,
        y: number,
        z: number
    },
    rotation:{
        x: number,
        y: number,
        z: number,
        w: number
    }
}

interface RobotStoreProps {
    robotPose: RobotPose,
    setRobotPose: (robotPose: RobotPose) => void
}

export const useRobotStore = create<RobotStoreProps>((set) => ({
    robotPose:{
        translation:{
            x: 0,
            y: 0,
            z: 0
        },
        rotation:{
            x: 0,
            y: 0,
            z: 0,
            w: 0
        }
    },
    setRobotPose: (robotPose : RobotPose) => set({robotPose}),
}));
