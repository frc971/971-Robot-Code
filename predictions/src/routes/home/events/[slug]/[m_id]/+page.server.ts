import { error } from '@sveltejs/kit';
import { db } from '$lib/firestore';
import { collection, addDoc, doc, getDoc } from 'firebase/firestore';
import { invalid } from '@sveltejs/kit';

/** @type {import('./$types').PageServerLoad} */
export async function load({ params }) {
	//Match ID
	const match_id = params.m_id;

	//Query Documents
	const docRef = doc(db, 'matches', match_id);
	const docSnap = await getDoc(docRef);

	const match = docSnap.data();

	if (match) {
		return {
			match: match,
			match_id: match_id
		};
	}

	throw error(404, 'Not found');
}
