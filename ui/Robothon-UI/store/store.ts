/**
 * @file robotStore.ts
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief Robot Store
 *
 * @copyright Copyright (c) 2024
 *
 */
import { create } from "zustand";

interface StoreProps {
    tasks : number[]
    setTasks : (tasks: number[]) => void
}

export const useStore = create<StoreProps>((set) => ({
    tasks : [],
    setTasks : (tasks : number[]) => set({tasks})
}))
