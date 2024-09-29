import { create } from "zustand";

interface StoreProps {
    tasks : number[]
    setTasks : (tasks: number[]) => void
}

export const useStore = create<StoreProps>((set) => ({
    tasks : [],
    setTasks : (tasks : number[]) => set({tasks})
}))
