import {add} from './basic';

describe('add', () => {
  it('should sum numbers', () => {
    expect(3 + 4).toEqual(7);
  });

  it('should not sum numbers', () => {
    expect(add(3, 3)).not.toEqual(7);
  });
});
