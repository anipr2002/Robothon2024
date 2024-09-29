import { create } from "zustand";
import { set_freedive_args ,get_gripper_calibration} from "@/utils/services"

type GripperArgProps = { position_unit: number, position: number, speed: number, force: number, asynchronous: boolean };


interface serviceStoreProps {
    gripper: GripperArgProps
    setGripper: (gripper: GripperArgProps) => void
    freedrive: set_freedive_args
    setFreedrive: (freedrive: set_freedive_args) => void
}

export const useServiceStore = create<serviceStoreProps>((set) => ({
    gripper: { position_unit: 0, position: 0.0, speed: 0.0, force: 0.0, asynchronous: false },
    setGripper: (gripper: GripperArgProps) => set({gripper}),
    setFreedrive: (freedrive: set_freedive_args) => set({freedrive}),
    freedrive: { free_axes:[1,1,1,1,1,1], IO: true, feature:0}
}))

get_gripper_calibration.callService({}, (result) => {
    console.log("/get_gripper_calibration called", result);
    
});
