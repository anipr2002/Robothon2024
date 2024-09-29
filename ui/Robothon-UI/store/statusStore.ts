/**
 * @file robotStore.ts
 * @authors Anirudh Panchangam Ranganath(anirudh.panchangamranganath@study.thws.de)\
 * @brief Robot Store
 *
 * @copyright Copyright (c) 2024
 *
 */
import {create } from 'zustand';


interface StatusStoreProps {
    connectedToROS : boolean;
    setConnectedToROS: (connected: boolean) => void;
    connectingToROS: boolean;
    setConnectingToROS: (connecting: boolean) => void;
}

export const useStatusStore = create<StatusStoreProps>((set) => ({
    connectedToROS: false,
    setConnectedToROS: (connected) => set({connectedToROS: connected}),
    connectingToROS: false,
    setConnectingToROS: (connecting) => set({connectingToROS: connecting}),
}));
