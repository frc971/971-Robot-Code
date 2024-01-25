import Dexie, {Table} from 'dexie';

export interface MatchListData {
  id?: number;
  data: Uint8Array;
}

export class AppDB extends Dexie {
  matchListData!: Table<MatchListData, number>;

  constructor() {
    super('ngdexieliveQuery');
    this.version(1).stores({
      matchListData: 'id,data',
    });
  }
}
export const db = new AppDB();
