import type { StopPoint, Constraint, Spline } from './type'

import type { Ui } from '../ui/ui'
type UiState = {
  splines: Spline[]
  rotbreakpoints: [number, number][]
  staticrotbreakpoints: number[]
  constraints: Constraint[]
  pathOutOfDate: boolean
  stopPoints: StopPoint[]
}


export class ChangeEvent {
  private previousState: UiState
  private nextState: UiState | null = null

  constructor(ui: Ui) {
    this.previousState = this.cloneState(ui)
  }

  public finalize(ui: Ui) {
    this.nextState = this.cloneState(ui)
  }

  public undo(ui: Ui) {
    this.applyState(ui, this.previousState)
  }

  public redo(ui: Ui) {
    if (this.nextState) {
      this.applyState(ui, this.nextState)
    }
  }

  private cloneState(ui: Ui): UiState {
    return {
      splines: structuredClone(ui.splines),
      rotbreakpoints: structuredClone(ui.rotbreakpoints),
      staticrotbreakpoints: structuredClone(ui.staticrotbreakpoints),
      constraints: structuredClone(ui.constraints),
      pathOutOfDate: ui.pathOutOfDate,
      stopPoints: structuredClone(ui.stopPoints),
    }
  }

  private applyState(ui: Ui, state: UiState) {
    ui.splines = state.splines
    ui.rotbreakpoints = state.rotbreakpoints
    ui.staticrotbreakpoints = state.staticrotbreakpoints
    ui.constraints = state.constraints
    ui.pathOutOfDate = state.pathOutOfDate
    ui.stopPoints = state.stopPoints
    if (ui.splines.length === 0) {
      ui.mode = "view"
    }
    ui.selectedContext.update()
  }
}


