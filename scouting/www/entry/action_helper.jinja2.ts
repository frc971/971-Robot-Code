import {
  ActionT,
  ActionType,
{% for action in ACTIONS %}
  {{ action }}T,
{% endfor %}
} from '@org_frc971/scouting/webserver/requests/messages/submit_2024_actions_generated';

export type ConcreteAction =
{% for action in ACTIONS %}
  {{ action }}T {% if not loop.last %} | {% endif %}
{% endfor %};

export class ActionHelper {
  constructor(
    private addAction: (actionType: ActionType, action: ConcreteAction) => void
  ){}

  {% for action in ACTIONS %}
  // Calls `addAction` in entry.component.ts with the proper arguments. This
  // also forces users to specify all the attributes in the `action` object.
  public add{{ action}}(action: NonFunctionProperties<{{ action }}T>): void {
    this.addAction(ActionType.{{ action }}, Object.assign(new {{ action }}T(), action));
  }
  {% endfor %}
}

type NonFunctionPropertyNames<T> = {
  [K in keyof T]: T[K] extends Function ? never : K
}[keyof T];

type NonFunctionProperties<T> = Pick<T, NonFunctionPropertyNames<T>>;
