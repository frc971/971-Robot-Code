import type { Ui } from './ui'
import type { StopPoint, Constraint, GlobalConstraint, ActionInfo, Rotation } from './type'
import { ChangeEvent } from './changeEvent'

type ContextInfo = {
  name: string,
  min?: number,
  max: number | null,
  step: number, // forces the value to be a multiple of step
  default: number | null,
  placeholder?: string, // gray text for when the field is empty
  allow_null: boolean, // is the value allowed to be empty? if not, we force a value if the user tries to make it empty
  onchange: (newval: number) => void
}

export abstract class Context {
  public parameter_list: ContextInfo[] = []
  public button_list: { name: string, action: (ev: MouseEvent) => void }[] = []
  public string_parameter_list: { name: string, default: string, onchange: (ev: Event) => void }[] = []
  protected ui: Ui

  public abstract update(): void
}

export class StopPointContext extends Context {
  public stop_point: StopPoint

  public constructor(values: StopPoint, ui: Ui) {
    super()

    this.ui = ui
    this.stop_point = values

    this.update()
  }

  public update() {
    let plist: ContextInfo[] = []
    let this_ = this
    plist.push({
      name: "delay",
      max: null,
      min: 0,
      default: this.stop_point.delay,
      step: 0.01,
      allow_null: false,
      onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.stop_point.delay = newval
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    this.parameter_list = plist
  }
}

export class ConstraintContext extends Context {
  public constraint: Constraint

  public constructor(values: Constraint, ui: Ui) {
    super()

    this.ui = ui
    this.constraint = values

    this.update()
  }

  public update() {
    let plist: ContextInfo[] = []
    let this_ = this
    plist.push({
      name: "selection start",
      max: 1,
      min: 0,
      default: this.constraint.selection[0],
      step: 0.001,
      allow_null: false,
      onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.constraint.selection[0] = newval
        this_.ui.updateCanvas()
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    plist.push({
      name: "selection end",
      max: 1,
      min: 0,
      default: this.constraint.selection[1],
      step: 0.001,
      allow_null: false,
      onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.constraint.selection[1] = newval
        this_.ui.updateCanvas()
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    plist.push({
      name: "maximum current",
      max: null,
      min: 0,
      default: this.constraint.max_current,
      step: 0.05,
      allow_null: true,
      onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.constraint.max_current = newval
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    plist.push({
      name: "maximum voltage",
      max: null,
      min: 0,
      default: this.constraint.max_voltage,
      step: 0.05,
      allow_null: true,
      onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.constraint.max_voltage = newval
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    plist.push({
      name: "maximum velocity",
      max: null,
      min: 0,
      default: this.constraint.max_velocity,
      step: 0.05,
      allow_null: true,
      onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.constraint.max_velocity = newval
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    plist.push({
      name: "maximum acceleration",
      max: null,
      min: 0,
      default: this.constraint.max_acceleration,
      step: 0.05,
      allow_null: true,
      onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.constraint.max_acceleration = newval
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    this.button_list.push({
      name: "delete",
      action(ev) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.ui.removeConstraint(this_.constraint)
        this_.ui.updateCanvas()
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    this.parameter_list = plist
  }
}


export class GlobalContext extends Context {
  public global_constraint: GlobalConstraint

  public constructor(values: GlobalConstraint, ui: Ui) {
    super()

    this.ui = ui
    this.global_constraint = values

    this.update()
  }

  public update() {
    let plist: ContextInfo[] = []
    let this_ = this
    plist.push({
      name: "maximum current",
      max: null,
      min: 0,
      default: this.global_constraint.max_current,
      step: 0.05,
      allow_null: true,
      onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.global_constraint.max_current = newval
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    plist.push({
      name: "maximum voltage",
      max: null,
      min: 0,
      default: this.global_constraint.max_voltage,
      step: 0.05,
      allow_null: true,
      onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.global_constraint.max_voltage = newval
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    plist.push({
      name: "maximum velocity",
      max: null,
      min: 0,
      allow_null: true,
      default: this.global_constraint.max_velocity,
      step: 0.05, onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.global_constraint.max_velocity = newval
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    plist.push({
      name: "maximum acceleration",
      max: null,
      min: 0,
      default: this.global_constraint.max_acceleration,
      step: 0.05,
      allow_null: true,
      onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.global_constraint.max_acceleration = newval
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
      }
    })
    this.parameter_list = plist
  }
}

export class ActionContext extends Context {
  public action: ActionInfo

  public constructor(values: ActionInfo, ui: Ui) {
    super()

    this.ui = ui
    this.action = values

    this.update()
  }

  public update() {
    let plist: ContextInfo[] = []
    let this_ = this
    plist.push({
      name: "location",
      max: 1,
      min: 0,
      default: this.action.location,
      step: 0.001,
      allow_null: false,
      onchange(newval) {
        this_.action.location = newval
        this_.ui.updateCanvas()
      },
    })
    this.string_parameter_list = [
      {
        name: "event name",
        default: this_.action.name,
        onchange(newval) {
          this_.ui.changeInProgress = new ChangeEvent(this_.ui)
          this_.action.name = (newval.target as HTMLInputElement).value
          this_.ui.updateCanvas()
          this_.ui.finalizeChange()
        },
      }
    ]
    this.button_list.push({
      name: "delete",
      action(ev) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.ui.removeAction(this_.action)
        this_.ui.updateCanvas()
        this_.ui.finalizeChange()
      }
    })
    this.parameter_list = plist
  }
}

export class RotationContext extends Context {
  public rotation: Rotation
  public fixed: boolean

  public constructor(values: Rotation, fixed: boolean, ui: Ui) {
    super()

    this.ui = ui
    this.rotation = values
    this.fixed = fixed

    this.update()
  }

  public update() {
    let plist: ContextInfo[] = []
    let this_ = this
    if(!this.fixed) {
      plist.push({
        name: "location",
        max: 1,
        min: 0,
        default: this.rotation[0],
        step: 0.001,
        allow_null: false,
        onchange(newval) {
          this_.ui.changeInProgress = new ChangeEvent(this_.ui)
          this_.rotation[0] = newval
          this_.ui.pathOutOfDate = true
          this_.ui.finalizeChange()
          this_.ui.updateCanvas()
        }
      })
      this.button_list.push({
        name: "delete",
        action(ev) {
          this_.ui.changeInProgress = new ChangeEvent(this_.ui)
          this_.ui.removeRotation(this_.rotation)
          this_.ui.updateCanvas()
          this_.ui.pathOutOfDate = true
          this_.ui.finalizeChange()
        }
      })
    }
    plist.push({
      name: "angle",
      max: null,
      min: null,
      default: this.rotation[1] / Math.PI * 180,
      step: 0.1,
      allow_null: false,
      onchange(newval) {
        this_.ui.changeInProgress = new ChangeEvent(this_.ui)
        this_.rotation[1] = newval / 180 * Math.PI
        this_.ui.pathOutOfDate = true
        this_.ui.finalizeChange()
        this_.ui.updateCanvas()
      }
    })
    this.parameter_list = plist
  }
}
