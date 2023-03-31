import { error } from '@sveltejs/kit';
import { db } from '$lib/firestore';
import { collection, query, where, getDocs, doc, getDoc } from 'firebase/firestore';
import type { DocumentData } from 'firebase/firestore';

/** @type {import('./$types').PageServerLoad} */
export async function load({ params }) {
	const uid = params.uid;

	const q = query(collection(db, 'predictions'), where('uid', '==', uid));
	const querySnapshot = await getDocs(q);

	let predictions: DocumentData[] = [];

	querySnapshot.forEach((doc) => {
		predictions.push(doc.data());
	});

	let matches: DocumentData[] = [];

	for (let i = 0; i < predictions.length; i++) {
		const docRef = doc(db, 'matches', predictions[i].match);
		const docSnap = await getDoc(docRef);
		matches.push(docSnap.data());
	}

	return {
		predictions: predictions,
		matches: matches
	};
}
