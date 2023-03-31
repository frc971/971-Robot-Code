import { error } from '@sveltejs/kit';
import { db } from '$lib/firestore';
import { doc, getDoc } from 'firebase/firestore';
import { uid } from '$lib/store';
import { get } from 'svelte/store';

/** @type {import('./$types').LayoutServerLoad} */
export async function load() {
	let user_id: string;
	uid.subscribe((value) => {
		user_id = value;
	});

	//Query Admin Role
	const docRef = doc(db, 'roles', 'admin');
	const adminDoc = await getDoc(docRef);

	let admin = false;
	if (adminDoc.exists()) {
		const adminData = adminDoc.data();
		if (adminData.uid.includes(user_id)) {
			admin = true;
		}
	}

	const pointsRef = doc(db, 'points', user_id);
	const pointsSnap = await getDoc(pointsRef);

	return {
		admin: admin,
		points: pointsSnap.data()
	};
}
